# HC16_Driver
Huion HC16 Driver

Beta version, only for 4.5.*+ kernel versions

```make & sudo make install```
```sudo make uninstall```

# Attention!
1. This driver conflict with module "uclogic", please rmmod uclogic before load this module.
2. USB_DEVICE_ID_HUION_HS64(**0x006d**) is not in special driver list(**hid_have_special_driver**), we need to modify the hid kernel source to add it. This list is defined in drivers/hid/**hid-quirks.c** for linux-kernel >= 4.16 and drivers/hid/**hid-core.c** for linux-kernel < 4.16

# Keys 
1. For keyboard: hardcoded as RIGHTCTRL+RIGHTALT+[hc16_keys] for tablet keys <br>
    hc16_keys:
    - HC16_KEY_TOP_1: KEY_BACKSLASH
    - HC16_KEY_TOP_2: KEY_APOSTROPHE
    - HC16_KEY_TOP_3: KEY_COMMA
    - HC16_KEY_TOP_4: KEY_DOT
    - HC16_KEY_TOP_5: KEY_LEFTBRACE
    - HC16_KEY_TOP_6: KEY_RIGHTBRACE
    - HC16_KEY_BOTTOM_1: KEY_F13
    - HC16_KEY_BOTTOM_2: KEY_F14
    - HC16_KEY_BOTTOM_3: KEY_F15
    - HC16_KEY_BOTTOM_4: KEY_F16
    - HC16_KEY_BOTTOM_5: KEY_F17
    - HC16_KEY_BOTTOM_6: KEY_F18
2. The Center key is for switching abs/rel pos mode
3. For pen: <br>
if HC16_STYLUS_KYE_TYPE is 0, BTN_STYLUS & BTN_STYLUS2 for stylus. <br>
~~if HC16_STYLUS_KYE_TYPE is 1, BUTTON_MIDDLE & BUTTON_RIGHT for stylus.~~ <br>

