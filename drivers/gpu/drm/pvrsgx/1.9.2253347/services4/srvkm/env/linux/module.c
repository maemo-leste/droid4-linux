/*************************************************************************/ /*!
@Title          Linux module setup
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <linux/version.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38))
#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif
#endif

#define PVR_USE_PRE_REGISTERED_PLATFORM_DEV

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#if defined(SUPPORT_DRI_DRM)
#include <drm/drm_file.h>
#if defined(PVR_SECURE_DRM_AUTH_EXPORT)
#include "env_perproc.h"
#endif
#endif

#if defined(PVR_LDM_PLATFORM_MODULE)
#include <linux/platform_device.h>
#endif /* PVR_LDM_PLATFORM_MODULE */

#if defined(PVR_LDM_PCI_MODULE)
#include <linux/pci.h>
#endif /* PVR_LDM_PCI_MODULE */

#if defined(PVR_LDM_DEVICE_CLASS)
#include <linux/device.h>
#endif /* PVR_LDM_DEVICE_CLASS */

#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
#include <asm/uaccess.h>
#endif

#include "img_defs.h"
#include "services.h"
#include "kerneldisplay.h"
#include "kernelbuffer.h"
#include "syscommon.h"
#include "pvrmmap.h"
#include "mutils.h"
#include "mm.h"
#include "mmap.h"
#include "mutex.h"
#include "pvr_debug.h"
#include "srvkm.h"
#include "perproc.h"
#include "handle.h"
#include "pvr_bridge_km.h"
#include "proc.h"
#include "pvrmodule.h"
#include "private_data.h"
#include "lock.h"
#include "linkage.h"
#include "buffer_manager.h"

#if defined(SUPPORT_DRI_DRM)
#include "pvr_drm.h"
#endif
/*
 * DRVNAME is the name we use to register our driver.
 * DEVNAME is the name we use to register actual device nodes.
 */
#if defined(SUPPORT_DRI_DRM_EXTERNAL)
#define DRVNAME                PVRSRV_MODNAME
#define DEVNAME                PVRSRV_MODNAME
#else
#if defined(PVR_LDM_MODULE)
#define	DRVNAME		PVR_LDM_DRIVER_REGISTRATION_NAME
#endif
#define DEVNAME		PVRSRV_MODNAME
MODULE_SUPPORTED_DEVICE(DEVNAME);
#endif

/*
 * This is all module configuration stuff required by the linux kernel.
 */
#if defined(PVRSRV_NEED_PVR_DPF)
#include <linux/moduleparam.h>
extern IMG_UINT32 gPVRDebugLevel;
module_param(gPVRDebugLevel, uint, 0644);
MODULE_PARM_DESC(gPVRDebugLevel, "Sets the level of debug output (default 0x7)");
#endif /* defined(PVRSRV_NEED_PVR_DPF) */

#if defined(CONFIG_ION_OMAP)
#include <linux/ion.h>
#include <linux/omap_ion.h>
extern struct ion_device *omap_ion_device;
struct ion_client *gpsIONClient;
EXPORT_SYMBOL(gpsIONClient);
#endif /* defined(CONFIG_ION_OMAP) */

/* PRQA S 3207 2 */ /* ignore 'not used' warning */
EXPORT_SYMBOL(PVRGetDisplayClassJTable);
EXPORT_SYMBOL(PVRGetBufferClassJTable);

PVRSRV_LINUX_MUTEX gPVRSRVLock;

/* PID of process being released */
IMG_UINT32 gui32ReleasePID;

#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
static IMG_UINT32 gPVRPowerLevel;
#endif

static PVRSRV_LINUX_MUTEX gsPMMutex;
static IMG_BOOL bDriverIsSuspended;
static IMG_BOOL bDriverIsShutdown;

/*!
******************************************************************************

 @Function		PVRSRVDriverShutdown

 @Description

 Suspend device operation for system shutdown.  This is called as part of the
 system halt/reboot process.  The driver is put into a quiescent state by 
 setting the power state to D3.

 @input pDevice - the device for which shutdown is requested

 @Return nothing

*****************************************************************************/
void PVRSRVDriverShutdown(LDM_DEV *pDevice)
{
	PVR_TRACE(("PVRSRVDriverShutdown(pDevice=%p)", pDevice));

	LinuxLockMutexNested(&gsPMMutex, PVRSRV_LOCK_CLASS_POWER);

	if (!bDriverIsShutdown && !bDriverIsSuspended)
	{
		/*
		 * Take the bridge mutex, and never release it, to stop
		 * processes trying to use the driver after it has been
		 * shutdown.
		 */
		LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

		(void) PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D3);
	}

	bDriverIsShutdown = IMG_TRUE;

	/* The bridge mutex is held on exit */
	LinuxUnLockMutex(&gsPMMutex);
}

/*!
******************************************************************************

 @Function		PVRSRVDriverSuspend

 @Description

 For 2.6 kernels:
 Suspend device operation.  We always get three calls to this regardless of
 the state (D1-D3) chosen.  The order is SUSPEND_DISABLE, SUSPEND_SAVE_STATE
 then SUSPEND_POWER_DOWN.  We take action as soon as we get the disable call,
 the other states not being handled by us yet.

 For MontaVista 2.4 kernels:
 This call gets made once only when someone does something like

	# echo -e -n "suspend powerdown 0" >/sys.devices/legacy/pvrsrv0/power

 The 3rd, numeric parameter (0) in the above has no relevence and is not
 passed into us.  The state parameter is always zero and the level parameter
 is always SUSPEND_POWER_DOWN.  Vive la difference!

 @input pDevice - the device for which resume is requested

 @Return 0 for success or <0 for an error.

*****************************************************************************/
int PVRSRVDriverSuspend(LDM_DEV *pDevice, pm_message_t state)
{
	int res = 0;
#if !(defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL) && !defined(SUPPORT_DRI_DRM))
	PVR_TRACE(( "PVRSRVDriverSuspend(pDevice=%p)", pDevice));

	LinuxLockMutexNested(&gsPMMutex, PVRSRV_LOCK_CLASS_POWER);

	if (!bDriverIsSuspended && !bDriverIsShutdown)
	{
		LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

		if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D3) == PVRSRV_OK)
		{
			/* The bridge mutex will be held until we resume */
			bDriverIsSuspended = IMG_TRUE;
		}
		else
		{
			LinuxUnLockMutex(&gPVRSRVLock);
			res = -EINVAL;
		}
	}

	LinuxUnLockMutex(&gsPMMutex);
#endif
	return res;
}


/*!
******************************************************************************

 @Function		PVRSRVDriverResume

 @Description

 Resume device operation following a lull due to earlier suspension.  It is
 implicit we're returning to D0 (fully operational) state.  We always get three
 calls to this using level thus: RESUME_POWER_ON, RESUME_RESTORE_STATE then
 RESUME_ENABLE.  On 2.6 kernels We don't do anything until we get the enable
 call; on the MontaVista set-up we only ever get the RESUME_POWER_ON call.

 @input pDevice - the device for which resume is requested

 @Return 0 for success or <0 for an error.

*****************************************************************************/
int PVRSRVDriverResume(LDM_DEV *pDevice)
{
	int res = 0;
#if !(defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL) && !defined(SUPPORT_DRI_DRM))
	PVR_TRACE(("PVRSRVDriverResume(pDevice=%p)", pDevice));

	LinuxLockMutexNested(&gsPMMutex, PVRSRV_LOCK_CLASS_POWER);

	if (bDriverIsSuspended && !bDriverIsShutdown)
	{
		if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D0) == PVRSRV_OK)
		{
			bDriverIsSuspended = IMG_FALSE;
			LinuxUnLockMutex(&gPVRSRVLock);
		}
		else
		{
			/* The bridge mutex is not released on failure */
			res = -EINVAL;
		}
	}

	LinuxUnLockMutex(&gsPMMutex);
#endif
	return res;
}

#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL) && !defined(SUPPORT_DRI_DRM)
/*
 * If PVR_LDM_PCI_MODULE is defined (and PVR_MANUAL_POWER_CONTROL is *NOT* defined),
 * the device can be suspended and resumed without suspending/resuming the
 * system, by writing values into the power/state sysfs file for the device.
 * To suspend:
 *	echo -n 2 > power/state
 * To Resume:
 *	echo -n 0 > power/state
 *
 * The problem with this approach is that the device is usually left
 * powered up; it is the responsibility of the bus driver to remove
 * the power.
 *
 * Defining PVR_MANUAL_POWER_CONTROL is intended to make it easier to
 * debug power management issues, especially when power is really removed
 * from the device.  It is easier to debug the driver if it is not being
 * suspended/resumed with the rest of the system.
 *
 * When PVR_MANUAL_POWER_CONTROL is defined, the following proc entry is
 * created:
 * 	/proc/pvr/power_control
 * The driver suspend/resume entry points defined below no longer suspend or
 * resume the device.  To suspend the device, type the following:
 * 	echo 2 > /proc/pvr/power_control
 * To resume the device, type:
 * 	echo 0 > /proc/pvr/power_control
 * 
 * The following example shows how to suspend/resume the device independently
 * of the rest of the system.
 * Suspend the device:
 * 	echo 2 > /proc/pvr/power_control
 * Suspend the system.  Then you should be able to suspend and resume
 * as normal.  To resume the device type the following:
 * 	echo 0 > /proc/pvr/power_control
 */

IMG_INT PVRProcSetPowerLevel(struct file *file, const IMG_CHAR *buffer, IMG_UINT32 count, IMG_VOID *data)
{
	IMG_CHAR data_buffer[2];
	IMG_UINT32 PVRPowerLevel;

	if (count != sizeof(data_buffer))
	{
		return -EINVAL;
	}
	else
	{
		if (copy_from_user(data_buffer, buffer, count))
			return -EINVAL;
		if (data_buffer[count - 1] != '\n')
			return -EINVAL;
		PVRPowerLevel = data_buffer[0] - '0';
		if (PVRPowerLevel != gPVRPowerLevel)
		{
			if (PVRPowerLevel != 0)
			{
				if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D3) != PVRSRV_OK)
				{
					return -EINVAL;
				}
			}
			else
			{
				if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D0) != PVRSRV_OK)
				{
					return -EINVAL;
				}
			}

			gPVRPowerLevel = PVRPowerLevel;
		}
	}
	return (count);
}

void ProcSeqShowPowerLevel(struct seq_file *sfile,void* el)	
{
	seq_printf(sfile, "%lu\n", gPVRPowerLevel);
}

#endif

/*!
******************************************************************************

 @Function		PVRSRVOpen

 @Description

 Release access the PVR services node - called when a file is closed, whether
 at exit or using close(2) system call.

 @input pInode - the inode for the file being openeded

 @input pFile - the file handle data for the actual file being opened

 @Return 0 for success or <0 for an error.

*****************************************************************************/
int PVRSRVOpen(struct drm_device unref__ *dev, struct drm_file *pFile)
{
	PVRSRV_FILE_PRIVATE_DATA *psPrivateData;
	IMG_HANDLE hBlockAlloc;
	int iRet = -ENOMEM;
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32PID;
#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
	PVRSRV_ENV_PER_PROCESS_DATA *psEnvPerProc;
#endif

	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

	ui32PID = OSGetCurrentProcessIDKM();

	if (PVRSRVProcessConnect(ui32PID, 0) != PVRSRV_OK)
		goto err_unlock;

#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
	psEnvPerProc = PVRSRVPerProcessPrivateData(ui32PID);
	if (psEnvPerProc == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: No per-process private data", __FUNCTION__));
		goto err_unlock;
	}
#endif

	eError = OSAllocMem(PVRSRV_OS_NON_PAGEABLE_HEAP,
						sizeof(PVRSRV_FILE_PRIVATE_DATA),
						(IMG_PVOID *)&psPrivateData,
						&hBlockAlloc,
						"File Private Data");

	if(eError != PVRSRV_OK)
		goto err_unlock;

#if defined (SUPPORT_SID_INTERFACE)
	psPrivateData->hKernelMemInfo = 0;
#else
	psPrivateData->hKernelMemInfo = NULL;
#endif
#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
	psPrivateData->psDRMFile = pFile;

	list_add_tail(&psPrivateData->sDRMAuthListItem, &psEnvPerProc->sDRMAuthListHead);
#endif
	psPrivateData->ui32OpenPID = ui32PID;
	psPrivateData->hBlockAlloc = hBlockAlloc;
	set_private(pFile, psPrivateData);
	iRet = 0;
err_unlock:	
	LinuxUnLockMutex(&gPVRSRVLock);
	return iRet;
}


/*!
******************************************************************************

 @Function		PVRSRVRelease

 @Description

 Release access the PVR services node - called when a file is closed, whether
 at exit or using close(2) system call.

 @input pInode - the inode for the file being released

 @input pFile - the file handle data for the actual file being released

 @Return 0 for success or <0 for an error.

*****************************************************************************/
void PVRSRVRelease(void *pvPrivData)
{
	PVRSRV_FILE_PRIVATE_DATA *psPrivateData;
	int err = 0;

	LinuxLockMutexNested(&gPVRSRVLock, PVRSRV_LOCK_CLASS_BRIDGE);

	psPrivateData = (PVRSRV_FILE_PRIVATE_DATA *)pvPrivData;

	if (psPrivateData != IMG_NULL)
	{
#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
		list_del(&psPrivateData->sDRMAuthListItem);
#endif

		if(psPrivateData->hKernelMemInfo)
		{
			PVRSRV_KERNEL_MEM_INFO *psKernelMemInfo;

			/* Look up the meminfo we just exported */
			if(PVRSRVLookupHandle(KERNEL_HANDLE_BASE,
								  (IMG_PVOID *)&psKernelMemInfo,
								  psPrivateData->hKernelMemInfo,
								  PVRSRV_HANDLE_TYPE_MEM_INFO) != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: Failed to look up export handle", __FUNCTION__));
				err = -EFAULT;
				goto err_unlock;
			}

			/* Tell the XProc about the export if required */
			if (psKernelMemInfo->sShareMemWorkaround.bInUse)
			{
				BM_XProcIndexRelease(psKernelMemInfo->sShareMemWorkaround.ui32ShareIndex);
			}

			/* This drops the psMemInfo refcount bumped on export */
			if(FreeMemCallBackCommon(psKernelMemInfo, 0,
									 PVRSRV_FREE_CALLBACK_ORIGIN_EXTERNAL) != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: FreeMemCallBackCommon failed", __FUNCTION__));
				err = -EFAULT;
				goto err_unlock;
			}
		}

		/* Usually this is the same as OSGetCurrentProcessIDKM(),
		 * but not necessarily (e.g. fork(), child closes last..)
		 */
		gui32ReleasePID = psPrivateData->ui32OpenPID;
		PVRSRVProcessDisconnect(psPrivateData->ui32OpenPID);
		gui32ReleasePID = 0;

		OSFreeMem(PVRSRV_OS_NON_PAGEABLE_HEAP,
				  sizeof(PVRSRV_FILE_PRIVATE_DATA),
				  psPrivateData, psPrivateData->hBlockAlloc);
	}

err_unlock:
	LinuxUnLockMutex(&gPVRSRVLock);
	return;
}


/*!
******************************************************************************

 @Function		PVRCore_Init

 @Description

 Insert the driver into the kernel.

 The device major number is allocated by the kernel dynamically.  This means
 that the device node (nominally /dev/pvrsrv) will need to be re-made at boot
 time if the number changes between subsequent loads of the module.  While the
 number often stays constant between loads this is not guaranteed.  The node
 is made as root on the shell with:

 		mknod /dev/pvrsrv c nnn 0

 where nnn is the major number found in /proc/devices for DEVNAME and also
 reported by the PVR_DPF() - look at the boot log using dmesg' to see this).

 Currently the auto-generated script /etc/init.d/rc.pvr handles creation of
 the device.  In other environments the device may be created either through
 devfs or sysfs.

 Readable proc-filesystem entries under /proc/pvr are created with
 CreateProcEntries().  These can be read at runtime to get information about
 the device (eg. 'cat /proc/pvr/vm')

 __init places the function in a special memory section that the kernel frees
 once the function has been run.  Refer also to module_init() macro call below.

 @input none

 @Return none

*****************************************************************************/
int PVRCore_Init(void)
{
	int error;
	PVRSRV_ERROR eError;

	PVR_TRACE(("PVRCore_Init"));

	LinuxInitMutex(&gsPMMutex);
	LinuxInitMutex(&gPVRSRVLock);

	if (CreateProcEntries ())
	{
		error = -ENOMEM;
		return error;
	}

	if (PVROSFuncInit() != PVRSRV_OK)
	{
		error = -ENOMEM;
		goto init_failed;
	}

	PVRLinuxMUtilsInit();

	if(LinuxMMInit() != PVRSRV_OK)
	{
		error = -ENOMEM;
		goto init_failed;
	}

	LinuxBridgeInit();

	PVRMMapInit();

	/*
	 * Drivers using LDM, will call SysInitialise in the probe/attach code
	 */
	if ((eError = SysInitialise()) != PVRSRV_OK)
	{
		error = -ENODEV;
#if defined(TCF_REV) && (TCF_REV == 110)
		if(eError == PVRSRV_ERROR_NOT_SUPPORTED)
		{
			printk("\nAtlas wrapper (FPGA image) version mismatch");
			error = -ENODEV;
		}
#endif
		goto init_failed;
	}

	return 0;

init_failed:
	PVRMMapCleanup();
	LinuxMMCleanup();
	LinuxBridgeDeInit();
	PVROSFuncDeInit();
	RemoveProcEntries();

	return error;

} /*PVRCore_Init*/


/*!
*****************************************************************************

 @Function		PVRCore_Cleanup

 @Description	

 Remove the driver from the kernel.

 There's no way we can get out of being unloaded other than panicking; we
 just do everything and plough on regardless of error.

 __exit places the function in a special memory section that the kernel frees
 once the function has been run.  Refer also to module_exit() macro call below.

 Note that the for LDM on MontaVista kernels, the positioning of the driver
 de-registration is the opposite way around than would be suggested by the
 registration case or the 2,6 kernel case.  This is the correct way to do it
 and the kernel panics if you change it.  You have been warned.

 @input none

 @Return none

*****************************************************************************/
void PVRCore_Cleanup(void)
{
	SYS_DATA *psSysData;
	PVR_TRACE(("PVRCore_Cleanup"));

	SysAcquireData(&psSysData);

#if defined(DEBUG) && defined(PVR_MANUAL_POWER_CONTROL)
	if (gPVRPowerLevel != 0)
	{
		if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D0) == PVRSRV_OK)
		{
			gPVRPowerLevel = 0;
		}
	}
#endif
	/* LDM drivers call SysDeinitialise during PVRSRVDriverRemove */
	(void) SysDeinitialise(psSysData);

	PVRMMapCleanup();

	LinuxMMCleanup();

	LinuxBridgeDeInit();

	PVROSFuncDeInit();

	RemoveProcEntries();

	PVR_TRACE(("PVRCore_Cleanup: unloading"));
}
