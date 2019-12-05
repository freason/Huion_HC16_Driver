# HC16_Driver
Huion HC16 Driver

Beta version, only for 4.5.*+ kernel versions

```make & make install```

# Attention!
1. This driver conflict with module "uclogic", please rmmod uclogic before load this module.
2. USB_DEVICE_ID_HUION_HS64(***0x006d***) is not in special driver list(***hid_have_special_driver***), we need to modify the hid kernel source to add it. This list is defined in drivers/hid/***hid-quirks.c*** for linux-kernel >= 4.16 and drivers/hid/***hid-core.c*** for linux-kernel < 4.16

# Keys 
1. For keyboard: hardcoded as RIGHTCTRL+RIGHTALT+[hc16_keys] for tablet keys
2. For pen: <br>
if HC16_STYLUS_KYE_TYPE is 0, BTN_STYLUS & BTN_STYLUS2 for stylus. <br>
if HC16_STYLUS_KYE_TYPE is 1, BUTTON_MIDDLE & BUTTON_RIGHT for stylus. <br>
`Stylus has hardware bug and not sent a keycode when pressure not null. `
