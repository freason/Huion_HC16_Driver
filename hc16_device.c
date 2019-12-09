#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include "compat.h"
#include <linux/version.h>
#include <linux/hid.h>
#include <linux/usb.h>
#include <linux/jiffies.h>
#include <asm/unaligned.h>
#include <stdbool.h>


#define	hid_to_usb_dev(hid_dev) container_of(hid_dev->dev.parent->parent, struct usb_device, dev)

#define MODULENAME                  "hc16_device"
#define DEVNAME                     "HUION Huion Tablet"

#define USB_VENDOR_ID_HUION               0x256c // 0x256c
#define USB_DEVICE_ID_HUION_HC16_TABLET   0x006d // 0x006e

#define CONFIG_BUF_SIZE 514

#define MAX_ABS_X 0xc670 // 50800
#define MAX_ABS_Y 0x7c06 // 31750
#define MAX_ABS_PRESSURE 0x1fff // 8191

#define REL_PEN_DIV 1
#define REL_PEN_POS_RESET_SKIP_COUNT 1
#define REL_PEN_UP_TICK 10

#define DEBUG
#define DPRINT(d, ...)       printk(d, ##__VA_ARGS__)
#define DPRINT_DEEP(d, ...)  //printk(d, ##__VA_ARGS__)

#define HC16_KEY_TOP_1 KEY_BACKSLASH
#define HC16_KEY_TOP_2 KEY_APOSTROPHE
#define HC16_KEY_TOP_3 KEY_COMMA
#define HC16_KEY_TOP_4 KEY_DOT
#define HC16_KEY_TOP_5 KEY_LEFTBRACE
#define HC16_KEY_TOP_6 KEY_RIGHTBRACE

#define HC16_KEY_BOTTOM_1 KEY_F13
#define HC16_KEY_BOTTOM_2 KEY_F14
#define HC16_KEY_BOTTOM_3 KEY_F15
#define HC16_KEY_BOTTOM_4 KEY_F16
#define HC16_KEY_BOTTOM_5 KEY_F17
#define HC16_KEY_BOTTOM_6 KEY_F18

#define HC16_KEY_CENTER   KEY_F22

struct hc16_devic_sc {
    struct input_dev *input;
    unsigned long quirks;
};

typedef unsigned short (*hc16_key_mapping_func_t)(u16 key_raw, unsigned short** last_key_pp);

typedef struct __tag_relative_pen_t
{
    bool enabled;
    int last_x;
    int last_y;

    int origin_x;
    int origin_y;

    int reseting_count;

    u64 last_jiffies;
} relative_pen_t;

typedef struct __tag_hc16_device_t {
    struct input_dev* pen;
    struct input_dev* pad;
} hc16_device_t;

static unsigned short def_keymap[] = {
    HC16_KEY_TOP_1,
    HC16_KEY_TOP_2,
    HC16_KEY_TOP_3,
    HC16_KEY_TOP_4,
    HC16_KEY_TOP_5,
    HC16_KEY_TOP_6,
    HC16_KEY_BOTTOM_1,
    HC16_KEY_BOTTOM_2,
    HC16_KEY_BOTTOM_3,
    HC16_KEY_BOTTOM_4,
    HC16_KEY_BOTTOM_5,
    HC16_KEY_BOTTOM_6,
    HC16_KEY_CENTER,
    KEY_RIGHTCTRL,
    KEY_RIGHTALT,
};

static const unsigned int HC16_HwConnMasks[2] = {
#ifdef USE_STANDALONE_INPUT_DEVICE
    HID_CONNECT_HIDRAW, // for keyboard
    HID_CONNECT_HIDRAW  // for pen
#else   // no USE_STANDALONE_INPUT_DEVICE
    HID_CONNECT_HIDRAW, // for keyboard
    HID_CONNECT_DEFAULT  // for pen
#endif  // USE_STANDALONE_INPUT_DEVICE
};

static const int HC16_KeyMapSize = sizeof(def_keymap) / sizeof(def_keymap[0]);

struct input_dev* idev_pen = NULL;
struct input_dev* idev_keyboard = NULL;

static bool pen_pressed = false;
static bool stylus_pressed = false;
static bool stylus2_pressed = false;

static unsigned short last_key = 0;
static unsigned short last_vkey = 0;
static int last_wheel = 0;


static relative_pen_t rel_pen_data = {
    .enabled = false,  // enabled
    .last_x = -1, // last_x
    .last_y = -1, // last_y
    .origin_x = 0,  // origin_x
    .origin_y = 0,  // origin_y
    .reseting_count = 0,  // reseting_count
    .last_jiffies = 0   // last_jiffies
    };


static int hc16_probe(struct hid_device *hdev, const struct hid_device_id *id);

static int hc16_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size);

static int hc16_input_mapping(struct hid_device *hdev,
    struct hid_input *hi, 
    struct hid_field *field,
    struct hid_usage *usage, 
    unsigned long **bit, 
    int *max);

static int hc16_input_configured(struct hid_device *hdev, struct hid_input *hi);

static int hc16_register_pen(struct hid_device *hdev);
static int hc16_register_keyboard(struct hid_device *hdev, struct usb_device *usb_dev, bool usedNewName);
static void hc16_read_keyboard_strings(struct usb_device *usb_dev);

static void hc16_config_pen(struct input_dev* pen);
static void hc16_config_keyboard(struct input_dev* keyboard);

static void hc16_handle_wheel_event(struct input_dev* input, u8 b_key_raw);
static void hc16_handle_key_event(struct input_dev* input, u16 key_raw);
static void hc16_handle_pen_event(struct input_dev* input, u8 b_key_raw, int x_pos, int y_pos, int pressure);

static void hc16_handle_key_mapping_event(
    struct input_dev* input, 
    unsigned short keys[],
    int keyc,
    u16 key_raw,
    hc16_key_mapping_func_t kmp_func);
static unsigned short hc16_mapping_keys(u16 key_raw, unsigned short** last_key_pp);

static void hc16_report_keys(struct input_dev* input, const int keyc, const unsigned short* keys, int s);

static void hc16_calculate_pen_data(const u8* data, int* x_pos, int* y_pos, int* pressure);

static void hc16_relative_pen_toggle(void);
static bool hc16_relative_pen_is_enabled(void);
static void hc16_relative_pen_enable(void);
static void hc16_relative_pen_disable(void);
static void hc16_relative_pen_reset_origin(void);
static void hc16_relative_pen_update_origin(int x, int y);
static void hc16_relative_pen_check_and_try_reset_last_abs_pos(void);
static void hc16_relative_pen_reset_last_abs_pos(void);
static void hc16_relative_pen_limit_xy(int* xp, int* yp);
static void hc16_relative_pen_update_last_abs_pos(int x, int y);
static void hc16_relative_pen_get_rel_pos(int abs_x, int abs_y, int* rel_x, int* rel_y);

static int hc16_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
    int rc = 0;
    struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
    unsigned long driver_data = id->driver_data;
    int if_number = intf->cur_altsetting->desc.bInterfaceNumber;
    // struct hc16_devic_sc *hc16 = NULL;
    unsigned int hw_start_mask = HC16_HwConnMasks[if_number];

    
    hdev->quirks |= HID_QUIRK_MULTI_INPUT;
    // hdev->quirks |= HID_QUIRK_NO_EMPTY_INPUT;

    DPRINT("hc16 device hdev=%p, detected if=%d, product=%x", hdev, if_number, id->product);
    
    // hc16 = devm_kzalloc(&hdev->dev, sizeof(*hc16), GFP_KERNEL);
    // hid_set_drvdata(hdev, hc16);
    
    if (id->product == USB_DEVICE_ID_HUION_HC16_TABLET) {
        hid_set_drvdata(hdev, (void *)driver_data);

        rc = hid_parse(hdev);
        if (rc)
        {
            hid_err(hdev, "parse failed\n");
            return rc;
        }
        
        rc = hid_hw_start(hdev, hw_start_mask);
        if (rc)
        {
            hid_err(hdev, "hw start failed\n");
            return rc;
        }
        
        DPRINT("hc16_probe: if: %d, :%x", if_number, hdev->claimed);


        if (if_number == 1)
        {
            rc = hc16_register_pen(hdev);
        }
        else if (if_number == 0)
        {
            struct usb_device *usb_dev = interface_to_usbdev(intf);
            rc = hc16_register_keyboard(hdev, usb_dev, false);
        }

        if (rc != 0)
        {
            goto error_stop_hw;
        }

        DPRINT("hc16 device ok");
    }
    else
    {
        DPRINT("hc16 hid strange error");
        return -ENODEV;
    }

    return 0;
    
error_stop_hw:
    hid_hw_stop(hdev);
    return rc;
}

static int hc16_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int size)
{
    int pressure, x_pos, y_pos;

    if ((idev_keyboard == NULL) || (idev_pen == NULL)) return -ENODEV;

    DPRINT_DEEP("hc16_raw_event: %d\t%*phC", size, size, data);

    if (size == 12 && data[0] == 0x08)
    {
        uint8_t codeH = data[1] & 0xf0;

        switch (codeH)
        {
            case 0x80:  // pen
            {
                hc16_calculate_pen_data(data, &x_pos, &y_pos, &pressure);
                hc16_handle_pen_event(idev_pen, data[1], x_pos, y_pos, pressure);
                return 1;
            }
            case 0xE0:  // key
            {
                if (data[2] == 0x01 && data[3] == 0x01)
                {
                    u16 key = data[4] << 8 | data[5];
                    hc16_handle_key_event(idev_keyboard, key);
                }

                return 1;
            }
            case 0xF0:  // wheel
            {
                if (data[2] == 0x01 && data[3] == 0x01)
                {
                    hc16_handle_wheel_event(idev_keyboard, data[5]);
                }
                return 1;
            }
            default:;
        }
    }

    return 0;
}

static int hc16_input_mapping(struct hid_device *hdev,
    struct hid_input *hi, 
    struct hid_field *field,
    struct hid_usage *usage, 
    unsigned long **bit, 
    int *max)
{
    DPRINT("hc16_input_mapping: app: %d, usage{hid: %d, code: %d, type: %d}", 
           field->application, 
           usage->hid,
           usage->code,
           usage->type
          );
    return 0;
}

static int hc16_input_configured(struct hid_device *hdev, struct hid_input *hi)
{
    struct hid_field* field = NULL;
    
    DPRINT("hc16_input_configured");
    
    field = hi->report->field[0];
    
    // HID_GD_KEYBOARD, HID_GD_MOUSE, HID_GD_KEYPAD will be ignored
    switch (field->application) 
    {
    case HID_DG_PEN:
        hi->input->name = "Huion HC16 Pen";
        idev_pen = hi->input;
        hc16_config_pen(hi->input);
        DPRINT("hc16_input_configured: Pen");
        break;
    default:
        DPRINT("hc16_input_configured unexpected: %d", field->application);
    }

    return 0;
}

static int hc16_register_pen(struct hid_device *hdev)
{
#ifdef USE_STANDALONE_INPUT_DEVICE
    int rc;
    rc = hid_hw_open(hdev);
    if (rc)
    {
        hid_err(hdev, "hc16_register_pen: cannot open hidraw\n");
        return rc;
    }

    idev_pen = input_allocate_device();
    if (idev_pen == NULL)
    {
        hid_err(hdev, "failed to allocate input device for pen\n");
        return -ENOMEM;
    }

    input_set_drvdata(idev_pen, hdev);

    idev_pen->name       = "Huion HC16 Tablet Pen";
    idev_pen->id.bustype = BUS_USB;
    idev_pen->id.vendor  = 0x056a;
    idev_pen->id.version = 0;
    idev_pen->dev.parent = &hdev->dev;

    hc16_config_pen(idev_pen);

    rc = input_register_device(idev_pen);
    if (rc)
    {
        hid_err(hdev, "error registering the input device for pen\n");
        input_free_device(idev_pen);
        return rc;
    }
#else   // no USE_STANDALONE_INPUT_DEVICE
    struct hid_report *report;
    
    if (!(hdev->claimed & HID_CLAIMED_INPUT))
    {
        return -ENODEV;
    }
    
    report = hid_register_report(hdev, HID_INPUT_REPORT, 0x0a, 0);
    
    if (!report) {
        hid_err(hdev, "unable to register touch report\n");
        return -ENOMEM;
    }
#endif  // USE_STANDALONE_INPUT_DEVICE
    return 0;
}

static int hc16_register_keyboard(struct hid_device *hdev, struct usb_device *usb_dev, bool usedNewName)
{
    int rc = 0;
    rc = hid_hw_open(hdev);
    if (rc)
    {
        hid_err(hdev, "hc16_register_pen: cannot open hidraw\n");
        return rc;
    }
    
    hc16_read_keyboard_strings(usb_dev);

    idev_keyboard = input_allocate_device();
    if (idev_keyboard == NULL)
    {
        hid_err(hdev, "failed to allocate input device [kb]\n");
        return -ENOMEM;
    }
    
    input_set_drvdata(idev_keyboard, hdev);

    idev_keyboard->name = "Huion HC16 Tablet Keyboard";
    if (usedNewName)
    {
        idev_keyboard->id.bustype = BUS_USB;
        idev_keyboard->id.vendor  = 0x056a;
        idev_keyboard->id.version = 0;
        idev_keyboard->dev.parent = &hdev->dev;
    }
    else
    {
        idev_keyboard->id.bustype = hdev->bus;
        idev_keyboard->id.vendor  = hdev->vendor;
        idev_keyboard->id.version = hdev->product;
        idev_keyboard->dev.parent = &hdev->dev;
    }

    hc16_config_keyboard(idev_keyboard);

    rc = input_register_device(idev_keyboard);
    if (rc)
    {
        hid_err(hdev, "error registering the input device [kb]\n");
        input_free_device(idev_keyboard);
        return rc;
    }

    return 0;
}

static void hc16_read_keyboard_strings(struct usb_device *usb_dev)
{
    int rc = 0;
    char buf[CONFIG_BUF_SIZE];
    rc = usb_string(usb_dev, 0x02, buf, CONFIG_BUF_SIZE);
    if (rc > 0) DPRINT("String(0x02) = %s", buf);

    rc = usb_string(usb_dev, 0xc9, buf, 256);
    if (rc > 0) DPRINT("String(0xc9) = %s", buf);

    rc = usb_string(usb_dev, 0xc8, buf, 256);
    if (rc > 0) DPRINT("String(0xc8) = %s", buf);

    rc = usb_string(usb_dev, 0xca, buf, 256);
    if (rc > 0) DPRINT("String(0xca) = %s", buf);
}


static void hc16_config_pen(struct input_dev* pen)
{
    __clear_bit(BTN_LEFT, pen->keybit);
    __clear_bit(BTN_RIGHT, pen->keybit);
    __clear_bit(BTN_MIDDLE, pen->keybit);
    
    set_bit(EV_REP, pen->evbit);

    input_set_capability(pen, EV_ABS, ABS_X);
    input_set_capability(pen, EV_ABS, ABS_Y);
    input_set_capability(pen, EV_ABS, ABS_PRESSURE);
    input_set_capability(pen, EV_KEY, BTN_TOOL_PEN);

    input_set_capability(pen, EV_KEY, BTN_STYLUS);
    input_set_capability(pen, EV_KEY, BTN_STYLUS2);

    input_set_abs_params(pen, ABS_X, 0, MAX_ABS_X, 0, 0);
    input_set_abs_params(pen, ABS_Y, 0, MAX_ABS_Y, 0, 0);
    input_set_abs_params(pen, ABS_PRESSURE, 0, MAX_ABS_PRESSURE, 0, 0);
}

static void hc16_config_keyboard(struct input_dev* keyboard)
{
    int i = 0;
    keyboard->keycode              = def_keymap;
    keyboard->keycodemax           = HC16_KeyMapSize;
    keyboard->keycodesize          = sizeof(def_keymap[0]);

    input_set_capability(keyboard, EV_MSC, MSC_SCAN);

    input_set_capability(keyboard, EV_REL, REL_WHEEL);

    for (i=0; i<HC16_KeyMapSize; i++)
    {
        input_set_capability(keyboard, EV_KEY, def_keymap[i]);
    }
}

static void hc16_handle_wheel_event(struct input_dev* input, u8 b_key_raw)
{
    int d = (int)b_key_raw - last_wheel;
    int t_last_wheel = last_wheel;

    if (abs(d) > 0x0c / 2)
    {
      int s = d >= 0 ? 1 : -1;
      d -= s * 0x0c;
    }

    last_wheel = (int)b_key_raw;

    if (b_key_raw == 0 || t_last_wheel == 0)
    {
      return;
    }

    input_report_rel(input, REL_WHEEL, d);

    input_sync(input);
}

static void hc16_handle_key_event(struct input_dev* input, u16 key_raw)
{
    unsigned short keys[] = {
        KEY_RIGHTCTRL,
        KEY_RIGHTALT,
        0
    };
    int keyc = sizeof(keys) / sizeof(keys[0]);

    hc16_handle_key_mapping_event(input, keys, keyc, key_raw, hc16_mapping_keys);
}

static void hc16_handle_pen_event(struct input_dev* input, u8 b_key_raw, int x_pos, int y_pos, int pressure)
{
    int rpt_x = x_pos;
    int rpt_y = y_pos;

    bool penPressed = b_key_raw & 0x01;
    bool stylus1Pressed = b_key_raw & 0x02;
    bool stylus2Pressed = b_key_raw & 0x04;

    if (penPressed)
    {
        pen_pressed = true;
        input_report_key(input, BTN_TOOL_PEN, 1);
        input_report_abs(input, ABS_PRESSURE, pressure);
    }
    else
    {
        pen_pressed = false;
        input_report_key(input, BTN_TOOL_PEN, 0);
        input_report_abs(input, ABS_PRESSURE, 0);
    }

    if (stylus1Pressed)
    {
        if (!stylus_pressed)
        {
            input_report_key(input, BTN_STYLUS, 1);
            stylus_pressed = true;
        }
    }
    else
    {
        if (stylus_pressed)
        {
            input_report_key(input, BTN_STYLUS, 0);
            stylus_pressed = false;
        }
    }

    if (stylus2Pressed)
    {
        if (!stylus2_pressed)
        {
            input_report_key(input, BTN_STYLUS2, 1);
            stylus2_pressed = true;
        }
    }
    else
    {
        if (stylus2_pressed)
        {
            input_report_key(input, BTN_STYLUS2, 0);
            stylus2_pressed = false;
        }
    }

    if(hc16_relative_pen_is_enabled())
    {
        int rel_x = 0;
        int rel_y = 0;
        hc16_relative_pen_check_and_try_reset_last_abs_pos();
        hc16_relative_pen_get_rel_pos(x_pos, y_pos, &rel_x, &rel_y);

        rpt_x = rel_x + rel_pen_data.origin_x;
        rpt_y = rel_y + rel_pen_data.origin_y;

        hc16_relative_pen_limit_xy(&rpt_x, &rpt_y);
        hc16_relative_pen_update_origin(rpt_x, rpt_y);

        hc16_relative_pen_update_last_abs_pos(x_pos, y_pos);
    }

    DPRINT_DEEP("sensors: x=%08d y=%08d pressure=%08d", rpt_x, rpt_y, pressure);

    input_report_abs(input, ABS_X, rpt_x);
    input_report_abs(input, ABS_Y, rpt_y);
    input_sync(input);
}

static void hc16_handle_key_mapping_event(
    struct input_dev* input, 
    unsigned short keys[],
    int keyc,
    u16 key_raw,
    hc16_key_mapping_func_t kmp_func)
{
    unsigned short* rkey_p = keys + keyc - 1;
    int value = 1;
    unsigned short* last_key_p = NULL;
    unsigned short new_key = kmp_func(key_raw, &last_key_p);

    if (new_key == 0)
    {
        value = 0;
        new_key = *last_key_p;
    }

    if (last_key_p == &last_vkey && new_key == HC16_KEY_CENTER)
    {
        if (value != 0)
        {
            hc16_relative_pen_toggle();
            *last_key_p = new_key;
        }
        else
        {
            *last_key_p = 0;
        }
    }
    else
    {
        int t_last_key = *last_key_p;
        if (t_last_key != 0 && t_last_key != new_key && value != 0)
        {
            *rkey_p = t_last_key;
            hc16_report_keys(input, keyc, keys, 0);
        }

        if (new_key != KEY_UNKNOWN && new_key != 0)
        {
            *rkey_p = new_key;
            *last_key_p = new_key;
            hc16_report_keys(input, keyc, keys, value);
        }

        if (value == 0)
        {
            *last_key_p = 0;
        }
    }
}

static unsigned short hc16_mapping_keys(u16 key_raw, unsigned short** last_key_pp)
{
    *last_key_pp = &last_key;

    switch (key_raw)
    {
        case 0x0000:
        {
            return 0;
        }
        case 0x0100:
        {
            return HC16_KEY_TOP_1;
        }
        case 0x0200:
        {
            return HC16_KEY_TOP_2;
        }
        case 0x0400:
        {
            return HC16_KEY_TOP_3;
        }
        case 0x0800:
        {
            return HC16_KEY_TOP_4;
        }
        case 0x1000:
        {
            return HC16_KEY_TOP_5;
        }
        case 0x2000:
        {
            return HC16_KEY_TOP_6;
        }
        case 0x4000:
        {
            return HC16_KEY_BOTTOM_1;
        }
        case 0x8000:
        {
            return HC16_KEY_BOTTOM_2;
        }
        case 0x0001:
        {
            return HC16_KEY_BOTTOM_3;
        }
        case 0x0002:
        {
            return HC16_KEY_BOTTOM_4;
        }
        case 0x0004:
        {
            return HC16_KEY_BOTTOM_5;
        }
        case 0x0008:
        {
            return HC16_KEY_BOTTOM_6;
        }
        case 0x0010:
        {
            *last_key_pp = &last_vkey;
            return HC16_KEY_CENTER;
        }
        default:
        {
            return KEY_UNKNOWN;
        }
    }
}

static void hc16_report_keys(struct input_dev* input, const int keyc, const unsigned short* keys, int s)
{
    int i = 0;

    for (i = 0; i < keyc; ++i)
    {
        input_report_key(input, keys[i], s);
    }

    input_sync(input);
}

static void hc16_calculate_pen_data(const u8* data, int* x_pos, int* y_pos, int* pressure)
{
    *x_pos           = data[3] * 0xFF + data[2];
    *y_pos           = data[5] * 0xFF + data[4];
    *pressure        = data[7] * 0xFF + data[6];
}

static void hc16_relative_pen_toggle(void)
{
    if (!hc16_relative_pen_is_enabled())
    {
        hc16_relative_pen_enable();
    }
    else
    {
        hc16_relative_pen_disable();
    }
}

static bool hc16_relative_pen_is_enabled(void)
{
    return rel_pen_data.enabled;
}

static void hc16_relative_pen_enable(void)
{
    DPRINT("hc16_relative_pen_enable");
    hc16_relative_pen_reset_origin();
    hc16_relative_pen_reset_last_abs_pos();

    rel_pen_data.enabled = true;
}

static void hc16_relative_pen_disable(void)
{
    DPRINT("hc16_relative_pen_disable");
    rel_pen_data.enabled = false;
}

static void hc16_relative_pen_reset_origin(void)
{
    int cur_x = input_abs_get_val(idev_pen, ABS_X);
    int cur_y = input_abs_get_val(idev_pen, ABS_Y);

    hc16_relative_pen_update_origin(cur_x, cur_y);
}

static void hc16_relative_pen_update_origin(int x, int y)
{
    rel_pen_data.origin_x = x;
    rel_pen_data.origin_y = y;
}

static void hc16_relative_pen_limit_xy(int* xp, int* yp)
{
    int x = *xp;
    int y = *yp;

    if (x > MAX_ABS_X)
    {
        x = MAX_ABS_X;
        *xp = x;
    }
    else if (x < 0)
    {
        x = 0;
        *xp = x;
    }

    if (y > MAX_ABS_X)
    {
        y = MAX_ABS_Y;
        *yp = y;
    }
    else if (y < 0)
    {
        y = 0;
        *yp = y;
    }
}

static void hc16_relative_pen_check_and_try_reset_last_abs_pos(void)
{
    u64 cur_jiffies = get_jiffies_64();
    u64 dj = cur_jiffies - rel_pen_data.last_jiffies;

    if (dj > REL_PEN_UP_TICK)
    {
        hc16_relative_pen_reset_last_abs_pos();
    }
}

static void hc16_relative_pen_reset_last_abs_pos(void)
{
    rel_pen_data.last_x = -1;
    rel_pen_data.last_y = -1;
    rel_pen_data.reseting_count = REL_PEN_POS_RESET_SKIP_COUNT + 1;
}

static void hc16_relative_pen_update_last_abs_pos(int x, int y)
{
    if (rel_pen_data.reseting_count > 0)
    {
        --rel_pen_data.reseting_count;
    }

    rel_pen_data.last_x = x;
    rel_pen_data.last_y = y;
    rel_pen_data.last_jiffies = get_jiffies_64();
}

static void hc16_relative_pen_get_rel_pos(int abs_x, int abs_y, int* rel_x, int* rel_y)
{
    int dx = 0;
    int dy = 0;

    if (rel_pen_data.reseting_count > 0)
    {
        dx = 0;
        dy = 0;
    }
    else
    {
        dx = abs_x - rel_pen_data.last_x;
        dy = abs_y - rel_pen_data.last_y;
    }

    *rel_x = dx / REL_PEN_DIV;
    *rel_y = dy / REL_PEN_DIV;
}

#ifdef USE_STANDALONE_INPUT_DEVICE
static void __close_keyboard(struct input_dev** keyboard_p)
{
    struct input_dev* keyboard = *keyboard_p;

    if (keyboard != NULL)
    {
        input_unregister_device(keyboard);
        input_free_device(keyboard);
        *keyboard_p = NULL;
        DPRINT("HC16 keyboard unregistered");
    }
}

static void __close_pad(struct input_dev** pen_p)
{
    struct input_dev* pen = *pen_p;

    if (pen != NULL)
    {
        input_unregister_device(pen);
        input_free_device(pen);
        *pen_p =NULL;
        DPRINT("HC16 tab unregistered");
    }
}
#endif  // USE_STANDALONE_INPUT_DEVICE

void hc16_remove(struct hid_device *dev)
{
#ifdef USE_STANDALONE_INPUT_DEVICE
    struct usb_interface *intf = to_usb_interface(dev->dev.parent);
    if (intf->cur_altsetting->desc.bInterfaceNumber == 0) {
        __close_keyboard(&idev_keyboard);
    } else if (intf->cur_altsetting->desc.bInterfaceNumber == 1) {
        __close_pad(&idev_pen);
    }
    hid_hw_close(dev);
#endif  // USE_STANDALONE_INPUT_DEVICE
    hid_hw_stop(dev);
}

static const struct hid_device_id hc16_ids[] = {
    {
      HID_USB_DEVICE(USB_VENDOR_ID_HUION, USB_DEVICE_ID_HUION_HC16_TABLET),
      .driver_data = (kernel_ulong_t)NULL,
    },
    {}
};

static struct hid_driver hc16_driver = {
    .name           = MODULENAME,
    .id_table       = hc16_ids,
    .probe          = hc16_probe,
    .remove         = hc16_remove,
    .raw_event      = hc16_raw_event,
    .input_mapping      = hc16_input_mapping,
    .input_configured   = hc16_input_configured,
    // .report_fixup       = hc16_report_fixup,
};
module_hid_driver(hc16_driver);

MODULE_AUTHOR("Ferd Chen <freason@yeah.net>");
MODULE_DESCRIPTION("Huion HC16 device driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

MODULE_DEVICE_TABLE(hid, hc16_ids);
