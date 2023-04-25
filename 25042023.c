#include "lvgl/lvgl.h"
#include "lv_drivers/display/fbdev.h"
#include "lvgl/demos/lv_demos.h"
#include "lvgl/examples/lv_examples.h"
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <libsoc_gpio.h>
#include <poll.h>
#include <fcntl.h>


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>// for usleep
#include <string.h>

#include <net/if.h>
#include <linux/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define _POSIX_C_SOURCE 200809L

#define DEBUG 1

#if DEBUG
# define DEBUG_PRINT(x) printf x
#else
# define DEBUG_PRINT(x) do {} while (0)
#endif

//#include <fcntl.h> // for O_RDWR, O_NOCTTY, O_NDELAY
//#include <termios.h> // for termios, tcgetattr, tcsetattr, B9600, etc.
//#include "minmea/minmea.h"

/*/ Global objects*/
static lv_obj_t *tabview;
static lv_obj_t *tab_btns;
// static lv_group_t *groupBtn; // Group that can be traversed with a button (auto wrap around)
static int whichtab = 0;


/*/Main tab objects*/
static lv_obj_t *tab1;
static lv_obj_t *batteryUsage;
static lv_obj_t *powerUsage;
static lv_obj_t *currentUsage;
static lv_obj_t *voltage;
static lv_obj_t *meter;
static lv_obj_t *speedValue;

lv_meter_indicator_t * indic;

int inBattery; // Integer (percentage) that takes input from CAN or whatever -> Preferably with a simple function that calculates the value in %
float inPower; // Power in W from battery - calculated locally
int inCurrent; // Current current usage from battery?
float inCurrentFloat; // Current current usage converted to float after reading from CAN
int inVoltage; // Current voltage from battery 
float inVoltageFloat; // Current voltage converted to float after reading from CAN
int prevSpeed; // Previous speed
int speed; // Current speed
int temperature; // Current temperature of the battery
int error; // Error code
char float_text[32]; // Text buffer used for displaying floats on the screen

/*/ Bluetooth tab objects*/
static lv_obj_t *tab2;
static lv_obj_t *bluetoothBtn;

/*/ GPS tab objects*/
static lv_obj_t *tab3; // Tab 3 object for GPS information display

pthread_t gps_tid;
pthread_t rec_thread_id;
pthread_t send_thread_id;

// GPS  constants
//
//#define GPS_UART_DEVICE "/dev/ttyO4" // UART 
//#define GPS_BAUD_RATE B9600 //Baud rate for GPS module
//#define GPS_BUFFER_SIZE 1024 //Buffer size for GPS data 

//// GPS Thread 
//void* gps_thread(void* arg) {
//    struct termios uart_config; //UART config struct
//
//    int uart_fd = open(GPS_UART_DEVICE, O_RDWR | O_NOCTTY); // Open UART device
//    if (uart_fd < 0) {
//        perror("Eroare la deschiderea UART4");
//        return NULL;
//    }
//
//    // Config the UART
//    tcgetattr(uart_fd, &uart_config);
//    cfsetispeed(&uart_config, GPS_BAUD_RATE);
//    cfsetospeed(&uart_config, GPS_BAUD_RATE);
//    uart_config.c_cflag |= (CLOCAL | CREAD);
//    uart_config.c_cflag &= ~PARENB;
//    uart_config.c_cflag &= ~CSTOPB;
//    uart_config.c_cflag &= ~CSIZE;
//    uart_config.c_cflag |= CS8;
//
//    // Apply UART config
//    tcsetattr(uart_fd, TCSANOW, &uart_config);
//
//    // Loop to read GPS coordinate and speed 
//    while (1) {
//        gps_data_t gps_data;
//        if (gps_read_data(uart_fd, &gps_data) == 0) {
//            // Send GPS data to CAN-bus and update the menu
//            can_send_gps_data(&gps_data);
//            menu_update_gps_data(&gps_data);
//        }
//    }
//    close(uart_fd);
//    return NULL;
//}
//
//// GPS initialization function
//static void gps_init(void)
//{
//    gps_fd = open(GPS_UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
//    if (gps_fd == -1)
//    {
//        perror("GPS UART open");
//        exit(1);
//    }
//
// // Configure UART options
//    struct termios options;
//    tcgetattr(gps_fd, &options);
//    cfsetispeed(&options, GPS_BAUD_RATE);
//    cfsetospeed(&options, GPS_BAUD_RATE);
//    options.c_cflag |= (CLOCAL | CREAD);
//    options.c_cflag &= ~PARENB;
//    options.c_cflag &= ~CSTOPB;
//    options.c_cflag &= ~CSIZE;
//    options.c_cflag |= CS8;
//    options.c_cflag &= ~CRTSCTS;
//    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//    options.c_iflag &= ~(IXON | IXOFF | IXANY);
//    options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
//    options.c_oflag &= ~OPOST;
//    tcsetattr(gps_fd, TCSANOW, &options); // Apply the UART options
//}





#if LV_BUILD_EXAMPLES && LV_USE_LABEL && LV_USE_BTN && LV_USE_METER && LV_USE_MENU && LV_USE_USER_DATA && LV_USE_TABVIEW

#define DISP_BUF_SIZE (768 * 1024)
#define LV_MEME_SIZE    (104857600ul)




static void set_value_cb(void * indic, int32_t v)
{
    lv_meter_set_indicator_value(meter, indic, v);
}

static void set_value()
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_value_cb);
    lv_anim_set_var(&a, indic);
    lv_anim_set_values(&a, prevSpeed, speed);
    lv_anim_set_time(&a, 900);
    lv_anim_set_repeat_delay(&a, 100);
    lv_anim_set_playback_time(&a, 500);
    lv_anim_set_playback_delay(&a, 100);
    lv_anim_start(&a);
}

static void scroll_begin_event(lv_event_t * e)
{
    /*Disable the scroll animations. Triggered when a tab button is clicked */
    if(lv_event_get_code(e) == LV_EVENT_SCROLL_BEGIN) {
        lv_anim_t * a = lv_event_get_param(e);
        if(a)  a->time = 0;
    }
}


#endif

#define NUM_BUTTONS 4
int button_pins[NUM_BUTTONS] = {66, 67, 69, 68};
int button_pressed[NUM_BUTTONS] = {0}; // The state of each button (1 = pressed, 0 = not pressed)
long long last_button_time[NUM_BUTTONS] = {0};
//long last_button_time[NUM_BUTTONS] = {0};
#define DEBOUNCE_DELAY 0.01
#define MAX_BUF 64

typedef struct {
    gpio *gpio_pin;
    int button_index;
} Button;
// Declare a variable to hold the count value
uint32_t count = 0;
// to be deleted, declared in main
Button buttons[NUM_BUTTONS];


// Declare an input device interface object
lv_indev_t * button1;
lv_indev_t* label;

int read_button_state(void *arg);
//int my_callback(void *arg);

/* ----------------------------------------------------------------------------------
-----------------------------------CAN---------------------------------------------*/

int open_can_socket(const char *ifname) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        close(s);
        return -1;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return -1;
    }

    return s;
}

void *can_receive_thread(int s) {
    struct can_frame frame;

    while (1) {
        // Read the incoming CAN frame
        int nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            perror("read");
            pthread_exit(NULL);
        }

        // Process the received CAN frame
        DEBUG_PRINT(("Received CAN frame: ID = 0x%X, DLC = %d, Data = ", frame.can_id, frame.can_dlc));
        if (nbytes > 0) {
            switch (frame.can_id)
            {
                case 0x001:
                {
                    inVoltage = ((frame.data[0] << 8) | frame.data[1]);
                    inVoltageFloat = (float)inVoltage / 100;
                    DEBUG_PRINT(("inVoltage = 0x%X\n", inVoltage));

                    inCurrent = ((frame.data[2] << 8) | frame.data[3]);
                    inCurrentFloat = (float)inCurrent / 100;
                    DEBUG_PRINT(("inCurrent = 0x%X\n", inCurrent));

                    temperature = frame.data[4];
                    DEBUG_PRINT(("temperature = 0x%X\n", temperature)); 
                }
                break;
                case 0x002:
                {
                    prevSpeed = speed;
                    speed = ((frame.data[0] << 8) | frame.data[1]);
                    DEBUG_PRINT(("speed = 0x%X\n", speed));

                    error = frame.data[2];
                    DEBUG_PRINT(("error = 0x%X\n", error));

                    set_value();
                }
                break;
                case 0x123:
                {
                    DEBUG_PRINT(("button1 = 0x%X\n", frame.data[0]));
                    DEBUG_PRINT(("button2 = 0x%X\n", frame.data[1]));
                    DEBUG_PRINT(("button3 = 0x%X\n", frame.data[2]));
                    DEBUG_PRINT(("button4 = 0x%X\n", frame.data[3]));
                }
            }
        }
    }
}

void *can_send_thread(int s)
{
    uint8_t prev_button_state[NUM_BUTTONS] = {0};

    struct can_frame frame;

    while (1)
    {
        if (button_pressed[0] != prev_button_state[0] || 
        button_pressed[1] != prev_button_state[1] || 
        button_pressed[2] != prev_button_state[2] || 
        button_pressed[3] != prev_button_state[3])
        {
            frame.can_id = 0x123;
            frame.can_dlc = 4;
            frame.data[0] = button_pressed[0];
            frame.data[1] = button_pressed[1];
            frame.data[2] = button_pressed[2];
            frame.data[3] = button_pressed[3];
            // send power
            //frame.data[4] = power;

            if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
            {
                perror("write");
                exit(EXIT_FAILURE);
            }

            prev_button_state[0] = button_pressed[0];
            prev_button_state[1] = button_pressed[1];
            prev_button_state[2] = button_pressed[2];
            prev_button_state[3] = button_pressed[3];
        }

        usleep(1000000); // 1 s delay to prevent excessive CPU usage
    }
}
long long timeInMilliseconds(void) {
    struct timeval tv;

    gettimeofday(&tv,NULL);
    return (((long long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
}

int main(void)
{
    /* ----------------------------------------------------------------------------------
    -----------------------------------INIT--------------------------------------------*/
    /*LVGL init*/
    lv_init();

    /*Linux frame buffer device init*/
    fbdev_init();
    
   /*A small buffer for LittlevGL to draw the screen's content*/
    static lv_color_t buf[DISP_BUF_SIZE];

    /*Initialize a descriptor for the buffer*/
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, buf, NULL, DISP_BUF_SIZE);

    /*Initialize and register a display driver*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf   = &disp_buf;
    disp_drv.flush_cb   = fbdev_flush;
    disp_drv.hor_res    = 1024;
    disp_drv.ver_res    = 600;
    lv_disp_drv_register(&disp_drv);

    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x00000F), LV_PART_MAIN);
//     scr = lv.scr_act()
// scr.set_style_bg_color(lv.color_hex(0x0000F), lv.PART.MAIN)


    /* ----------------------------------------------------------------------------------
    -----------------------------------INIT--------------------------------------------*/

    /* ------------------------------- Init button devices ---------------------------------*/
 
    /*/static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);      //Basic initialization
    indev_drv.type = LV_INDEV_TYPE_BUTTON;   
    indev_drv.read_cb = button_read;         
    lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv); // Check laterr *TODO*  */
    


    // Gradient --------------------------------------------------
    static lv_style_t gradientStyle;
    lv_style_init(&gradientStyle);
    lv_style_set_radius(&gradientStyle, 10);

    /*Make a gradient*/
    lv_style_set_bg_opa(&gradientStyle, LV_OPA_COVER);
    static lv_grad_dsc_t grad;
    grad.dir = LV_GRAD_DIR_VER;
    grad.stops_count = 5;
    grad.stops[0].color = lv_palette_main(LV_PALETTE_GREY);
    grad.stops[1].color = lv_palette_darken(LV_PALETTE_GREY, 1);
    grad.stops[2].color = lv_palette_darken(LV_PALETTE_GREY, 2);
    grad.stops[3].color = lv_palette_darken(LV_PALETTE_GREY, 3);
    grad.stops[4].color = lv_palette_darken(LV_PALETTE_GREY, 4);

    /*Shift the gradient to the bottom*/
    grad.stops[0].frac  = 30;
    grad.stops[1].frac  = 94;
    grad.stops[2].frac  = 128;
    grad.stops[3].frac  = 192;
    grad.stops[4].frac  = 222;

    lv_style_set_bg_grad(&gradientStyle, &grad);

    // Shadow style --------------------------------------------

    // static lv_style_t shadowStyle;
    // lv_style_init(&shadowStyle);

    // /*Set a background color and a radius*/
    // lv_style_set_radius(&shadowStyle, 5);
    // lv_style_set_bg_opa(&shadowStyle, LV_OPA_60);
    // lv_style_set_bg_color(&shadowStyle, lv_palette_darken(LV_PALETTE_GREY, 4));

    /*Add a shadow*/
    // lv_style_set_shadow_width(&shadowStyle, 55);
    // lv_style_set_shadow_color(&shadowStyle, lv_palette_main(LV_PALETTE_BLUE));
    //    lv_style_set_shadow_ofs_x(&style, 10);
    //    lv_style_set_shadow_ofs_y(&style, 20);


    // ---------------------------------------------------------------

    //Button buttons[NUM_BUTTONS];

    // Initialize each button and register the interrupt handling function
    for (int i = 0; i < NUM_BUTTONS; i++) {
        buttons[i].gpio_pin = libsoc_gpio_request(button_pins[i], LS_GPIO_GREEDY);
        libsoc_gpio_set_direction(buttons[i].gpio_pin, INPUT);
        buttons[i].button_index = i;
        // libsoc_gpio_callback_interrupt(buttons[i].gpio_pin, &read_button_state, NULL);
        // libsoc_gpio_callback_interrupt(buttons[i].gpio_pin, &my_callback, &buttons[i]);

    }
    //static lv_style_t largerFontStyle;
    //lv_style_init(&largerFontStyle);
    //lv_style_set_text_font(&largerFontStyle, LV_FONT_MONTSERRAT_26);

    /*/Initialize group for traversing through tabs*/

    //lv_indev_set_group(indev, g) *TODO* Set up input device https://docs.lvgl.io/latest/en/html/porting/indev.html


    /*Create a Tab view object*/
    tabview = lv_tabview_create(lv_scr_act(), LV_DIR_BOTTOM, 80);
    lv_obj_add_event_cb(lv_tabview_get_content(tabview), scroll_begin_event, LV_EVENT_SCROLL_BEGIN, NULL);

    lv_obj_set_style_bg_color(tabview, lv_palette_darken(LV_PALETTE_GREY, 3), 0);

    tab_btns = lv_tabview_get_tab_btns(tabview);
    lv_obj_set_style_bg_color(tab_btns, lv_palette_darken(LV_PALETTE_DEEP_ORANGE, 3), 0);
    lv_obj_set_style_text_color(tab_btns, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
    lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_RIGHT, LV_PART_ITEMS | LV_STATE_CHECKED);


    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    tab1 = lv_tabview_add_tab(tabview, "Main");
    lv_obj_add_flag(tab1, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(tab1, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    //lv_obj_add_style(tab1, &gradientStyle, 0);
    tab2 = lv_tabview_add_tab(tabview, LV_SYMBOL_BLUETOOTH " Bluetooth");
    lv_obj_add_flag(tab2, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(tab2, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    //lv_obj_add_style(tab1, &gradientStyle, 0);
    tab3 = lv_tabview_add_tab(tabview, LV_SYMBOL_GPS " GPS");
    lv_obj_add_flag(tab3, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(tab3, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    //lv_obj_add_style(tab1, &gradientStyle, 0);
    /*/groupBtn = lv_group_create();
    lv_group_add_obj(groupBtn, tab1);
    lv_group_add_obj(groupBtn, tab2);
    lv_group_add_obj(groupBtn, tab3);

    lv_group_focus_obj(tab1);*/

    
    lv_obj_set_style_bg_color(tab2, lv_palette_lighten(LV_PALETTE_GREY, 5), 0);
    lv_obj_set_style_bg_opa(tab2, LV_OPA_40, 0);

    // TAB 1
    /*Add content to the tabs*/
    batteryUsage = lv_label_create(tab1);
    lv_label_set_text_fmt(batteryUsage, "Battery: %d %%", inBattery); // 
    lv_obj_align_to(batteryUsage, tab1, LV_ALIGN_TOP_LEFT, 60, 60);
    //lv_obj_add_style(batteryUsage, &largerFontStyle, 0);

    powerUsage = lv_label_create(tab1);
    lv_label_set_text_fmt(powerUsage, "Power: %d W", 0); // 
    lv_obj_align_to(powerUsage, tab1, LV_ALIGN_LEFT_MID, 60, 0);
    //lv_obj_add_style(powerUsage, &largerFontStyle, 0);

    currentUsage = lv_label_create(tab1);
    lv_label_set_text_fmt(currentUsage, "Current: %d A", inCurrent); // 
    lv_obj_align_to(currentUsage, tab1, LV_ALIGN_TOP_RIGHT, -60, 60);
    //lv_obj_add_style(currentUsage, &largerFontStyle, 0);

    voltage = lv_label_create(tab1);
    lv_label_set_text_fmt(voltage, "Voltage: %d V", inVoltage); //
    lv_obj_align_to(voltage, tab1, LV_ALIGN_RIGHT_MID, -60, 0);
    //lv_obj_add_style(voltage, &largerFontStyle, 0);

    lv_obj_clear_flag(lv_tabview_get_content(tabview), LV_OBJ_FLAG_SCROLLABLE);


    meter = lv_meter_create(tab1);
    lv_obj_center(meter);
    lv_obj_set_size(meter, 400, 400);




    /* Apply the gradient style to the meter */

    // lv_obj_add_style(meter, LV_PART_MAIN, &gradientStyle);

    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 8, 4, 15, lv_color_black(), 10);

    // lv_meter_indicator_t * indic;

    /*Add a blue arc to the start*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /*Make the tick lines blue at the start of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE),
                                    false, 0);
    lv_meter_set_indicator_start_value(meter, indic, 0);
    lv_meter_set_indicator_end_value(meter, indic, 20);

    /*Add a red arc to the end*/
    indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /*Make the tick lines red at the end of the scale*/
    indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false,
                                    0);
    lv_meter_set_indicator_start_value(meter, indic, 80);
    lv_meter_set_indicator_end_value(meter, indic, 100);

    /*Add a needle line indicator*/
    indic = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);

    /*Create an animation to set the value*/
    // lv_anim_t a;
    // lv_anim_init(&a);
    // lv_anim_set_exec_cb(&a, set_value);
    // lv_anim_set_var(&a, indic);
    // lv_anim_set_values(&a, 0, 100);
    // lv_anim_set_time(&a, 2000);
    // lv_anim_set_repeat_delay(&a, 100);
    // lv_anim_set_playback_time(&a, 500);
    // lv_anim_set_playback_delay(&a, 100);
    // lv_anim_set_repeat_count(&a, 1);
    // lv_anim_start(&a);

    // Add label under Meter
    static lv_obj_t *labelkmt;
    labelkmt = lv_label_create(meter);
    lv_label_set_text_static(labelkmt, "Km/h");
    lv_obj_align(labelkmt, LV_ALIGN_BOTTOM_MID, 0, 0);

    // Add speed value label under Meter
    speedValue = lv_label_create(tab1);
    lv_label_set_text_fmt(speedValue, "%d", 0);
    lv_obj_align_to(speedValue, labelkmt, LV_ALIGN_OUT_BOTTOM_MID, -15, -70);
    lv_obj_set_style_text_font(speedValue, LV_FONT_SPEED, 0);

    label = lv_label_create(lv_scr_act());
    lv_label_set_text_fmt(label, "%d", count);
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 0);

     // CAN Init
    const char *ifrecname = "can1";
    int can_rec_socket = open_can_socket(ifrecname);

     // Create the CAN receive thread
    pthread_t rec_thread_id;
    if (pthread_create(&rec_thread_id, NULL, (void *)can_receive_thread, can_rec_socket) != 0) {
        perror("pthread_create receive");
        return 1;
    }

    if (can_rec_socket < 0) {
        fprintf(stderr, "Failed to open CAN socket on interface %s\n", ifrecname);
        return 1;
    }

    DEBUG_PRINT(("CAN socket opened on interface %s\n", ifrecname));

    const char *ifsendname = "can0";
    int can_send_socket = open_can_socket(ifsendname);

    // Create the CAN send thread
    pthread_t send_thread_id;
    if (pthread_create(&send_thread_id, NULL, (void *)can_send_thread, can_send_socket) != 0) {
        perror("pthread_create send");
        return 1;
    }

    
    if (can_send_socket < 0) {
        fprintf(stderr, "Failed to open CAN socket on interface %s\n", ifsendname);
        return 1;
    }

    DEBUG_PRINT(("CAN socket opened on interface %s\n", ifsendname));

    //// / Create the GPS thread
    //if (pthread_create(& gps_data_t , NULL, gps_thread, NULL) < 0) {
    //    perror("Eroare la crearea thread-ului GPS");
    //    exit(1);
    //}

    // TAB 2

    bluetoothBtn = lv_btn_create(tab2);

    lv_obj_t * cont = lv_tabview_get_content(tab1);
    uint32_t tab_id = lv_obj_get_child_cnt(cont);
    DEBUG_PRINT(("%d", tab_id));

   
   
    while(1)
    {
    
        //Check the state of each button
        for (int i = 0; i < NUM_BUTTONS; i++) {
            if (timeInMilliseconds() - last_button_time[i] > 250)
            {
                if (libsoc_gpio_get_level(buttons[i].gpio_pin) == HIGH)
                {
                    button_pressed[i] = true;
                    DEBUG_PRINT(("Button %d was pressed\n", i+1));
                    last_button_time[i] = timeInMilliseconds();
                    count++;
                    lv_label_set_text_fmt(label, "%d", count);
                    
                    switch(whichtab)
                    {
                        case 0:
                            lv_tabview_set_act(tabview, whichtab, LV_ANIM_OFF);
                            whichtab = 1;
                            break;
                    
                        case 1:
                            lv_tabview_set_act(tabview, whichtab, LV_ANIM_OFF);
                            whichtab = 2;
                            break;

                        case 2:
                            lv_tabview_set_act(tabview, whichtab, LV_ANIM_OFF);
                            whichtab = 0;
                            break;
                        
                        default:
                            perror("tab nr not found??\n\n");
                            whichtab = 0;

                    }
                    uint16_t tab = lv_tabview_get_tab_act(tabview);
                    DEBUG_PRINT(("%d\n", tab));
                    DEBUG_PRINT(("whichtab == %d\n", whichtab));

                }
                else if (libsoc_gpio_get_level(buttons[i].gpio_pin) == LOW)
                {
                    button_pressed[i] = false;
                    // DEBUG_PRINT(("Button %d was relesed\n", i+1));
                    // last_button_time[i] = time(NULL);
                }
            }
        }

        // update km/h text
        lv_label_set_text_fmt(speedValue, "%d", speed);
        // update meter needle position
        // lv_meter_set_indicator_value(meter, indic, speed);

        snprintf(float_text, sizeof(float_text), "Voltage: %.2f V", inVoltageFloat);
        lv_label_set_text(voltage, float_text);
        snprintf(float_text, sizeof(float_text), "Current: %.2f A", inCurrentFloat);
        lv_label_set_text(currentUsage, float_text);
        inPower = inVoltageFloat * inCurrentFloat;
        snprintf(float_text, sizeof(float_text), "Power: %.2f W", inPower);
        lv_label_set_text(powerUsage, float_text);

        lv_scr_act();
        lv_tick_inc(5);
        lv_timer_handler();
        usleep(5000);
    }

    // Close CAN socket and thread
    pthread_join(rec_thread_id, NULL);
    pthread_join(send_thread_id, NULL);
    
    // Wait for the GPS thread to finish before releasing resources
    pthread_join(gps_tid, NULL);

    // Eliberați memoria alocată
    free(can_rec_socket);
    free(can_send_socket);

    close(can_rec_socket);
    close(can_send_socket);
}
//
//int gps_read_data(int uart_fd, gps_data_t *data) {
//    char buffer[256];
//    ssize_t bytes_read;
//
//    // Read the data from the GPS module
//    bytes_read = read(uart_fd, buffer, sizeof(buffer) - 1);
//    if (bytes_read < 0) {
//        return -1;
//    }
//
//    buffer[bytes_read] = '\0';
//
//    // Parse the NMEA sentences to extract GPS coordinates and speed
//    struct minmea_sentence_rmc rmc;
//    if (minmea_parse_rmc(&rmc, buffer) && rmc.valid) {
//        data->latitude = minmea_tocoord(&rmc.latitude);
//        data->longitude = minmea_tocoord(&rmc.longitude);
//        data->speed = minmea_tofloat(&rmc.speed);
//        return 0;
//    }
//    return -1;
//}
int read_button_state(void *arg)
{
    DEBUG_PRINT(("Interrupt\n"));
    for (int i = 0; i < NUM_BUTTONS; i++) {
            if (time(NULL) - last_button_time[i] > 0.01)
            {
                last_button_time[i] = time(NULL);

                if (libsoc_gpio_get_level(buttons[i].gpio_pin) == HIGH)
                {
                    DEBUG_PRINT(("Hello you've pressed me\n"));
                    button_pressed[i] = true;
                }
                else
                {
                    button_pressed[i] = false;
                }
            }
        }
}
