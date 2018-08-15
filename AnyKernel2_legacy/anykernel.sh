# AnyKernel2 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
# begin properties
properties() {
kernel.string=Venom by tanish2k09 @ xda-developers
do.devicecheck=1
do.modules=0
do.cleanup=1
do.cleanuponabort=0
device.name1=aio_row
device.name2=A7000
device.name3=A7000-a
device.name4=
device.name5=
do.system=1
} # end properties

# shell variables
block=/dev/block/platform/mtk-msdc.0/by-name/boot;
is_slot_device=0;
ramdisk_compression=auto;


## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. /tmp/anykernel/tools/ak2-core.sh;


## AnyKernel file attributes
# set permissions/ownership for included ramdisk files
chmod -R 750 $ramdisk_r/*;
chown -R root:root $ramdisk_r/*;


## AnyKernel install
dump_boot;
if [ -f "$legacy/legacy" ]; then
reset_ak;
dump_boot;
fi;


# begin ramdisk changes

# init.d patch
append_file init.rc "init.d support" init_patch;
append_file default.prop "sys.init.d.loop=on" init_prop_patch;
mkdir /system/etc/init.d
chmod -R 755 /system/etc/init.d
cp /tmp/anykernel/tools/busybox /system/xbin
chmod 755 /system/xbin/busybox

# adb secure
replace_string default.prop "ro.adb.secure=0" "ro.adb.secure=1" "ro.adb.secure=0";
replace_string default.prop "ro.secure=0" "ro.secure=1" "ro.secure=0";

# add spectrum support
insert_line init.mt6752.rc "import /init.spectrum.rc" after "import init.project.rc" "import /init.spectrum.rc\n";

# end ramdisk changes

write_boot;

## end install

