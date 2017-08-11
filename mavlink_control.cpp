/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include "px4_custom_mode.h"
#include <math.h>

enum takeoff_mode{
    TAKE_OFF_AUTOMATIC,
    TAKE_OFF_MANUAL_OR_GCS
};


static int takeoff_mode = TAKE_OFF_MANUAL_OR_GCS;
// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

    // --------------------------------------------------------------------------
    //   PARSE THE COMMANDS
    // --------------------------------------------------------------------------

    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
    char *uart_name = (char*)"/dev/ttyUSB0";
#endif
    int baudrate = 57600;

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate);


    // --------------------------------------------------------------------------
    //   PORT and THREAD STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate a serial port object
     *
     * This object handles the opening and closing of the offboard computer's
     * serial port over which it will communicate to an autopilot.  It has
     * methods to read and write a mavlink_message_t object.  To help with read
     * and write in the context of pthreading, it gaurds port operations with a
     * pthread mutex lock.
     *
     */
    Serial_Port serial_port(uart_name, baudrate);


    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Autopilot_Interface autopilot_interface(&serial_port);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    serial_port_quit         = &serial_port;
    autopilot_interface_quit = &autopilot_interface;
    signal(SIGINT,quit_handler);

    /*
     * Start the port and autopilot_interface
     * This is where the port is opened, and read and write threads are started.
     */
    serial_port.start();
    autopilot_interface.start();


    // --------------------------------------------------------------------------
    //   RUN COMMANDS
    // --------------------------------------------------------------------------

    /*
     * Now we can implement the algorithm we want on top of the autopilot interface
     */
    commands(autopilot_interface);


    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    autopilot_interface.stop();
    serial_port.stop();


    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    // woot!
    return 0;

}

float distance(float x1,float y1,float z1,float x2,float y2,float z2)
{
    float x, y,z;
    x = (x1-x2) * (x1-x2);
    y = (y1-y2) * (y1-y2);
    z = (z1-z2) * (z1-z2);
    return sqrt(x+y+z);
}

// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{
    Mavlink_Messages messages;

    // initialize command data strtuctures
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip = api.initial_position;

    // autopilot_interface.h provides some helper functions to build the command
   
    /**
    * Switch to Offboard mode Now
    */
    // messages = api.current_messages;    // copy current messages

    if(takeoff_mode == TAKE_OFF_MANUAL_OR_GCS){

        printf("###########################\n");
        printf("### Vehicle Should takeoff manually Already! ###\n");
        printf("###########################\n");
        
        //  Switch to Offboard mode
        printf("Waiting for Vehicle to be armed...\n");
        while(!api.is_armed()){
            usleep(100000);
        }
        
        printf("Vehicle is armed! Now switch to offboard mode...\n");
        sleep(5);

        api.enable_offboard_control();

        usleep(100); // give some time to let it sink in

        printf("Vehicle is armed and has switch to offboard mode!\n");
    } else if (takeoff_mode == TAKE_OFF_AUTOMATIC) {

        printf("Vehicle will auto takeoff!\n");
        /* 
         * Armed and tongle offboard control
         *       This is used in automatic take-off situation
        */

        //   START OFFBOARD MODE
        api.enable_offboard_control();
        printf("######## Offboard Mode ########## !\n");

        //   ARMED
        api.vehicle_armed();
        printf("######## Armed ########## !\n");

        usleep(100); // give some time to let it sink in
    }/*End: switch to Offboard mode*/

    /** 
    * Now the autopilot is waiting for accepting setpoint commands
    */
    int interval = 0;
    int waiting_interval = 0;
    printf("Waiting for 1 minute %2ds", waiting_interval);
    fflush(stdout);

    for(interval = waiting_interval; interval > 0; interval--){
        sleep(1);
        printf("\b\b\b%2ds", interval);

        fflush(stdout);
        
    }
    printf("\n");

    printf("SEND OFFBOARD COMMANDS\n");

    // Example 1 - Set Velocity
    set_velocity( 0.55     , // [m/s]
                            0.55      , // [m/s]
                            0.55     , // [m/s]
                            sp        );
    api.update_setpoint(sp);  // THEN pixhawk will try to move
    sleep(3);

    // // Example 2 - Set Position
    // ip.z: [unit - m] NOTE: Negative value will make vehicle fight up, Positive value will make it fight down;
    // set_position_velocity( 0.01, 0.01, 0.01, ip.x, ip.y, ip.z - 2.5, sp);
    set_position( ip.x, ip.y, ip.z - 2.5, sp);
    api.update_setpoint(sp);  // THEN pixhawk will try to move
    sleep(1);
    // set_yaw( ip.yaw + 1.57 /*[rad]-[90 dgree]*/, sp  );
    // set_velocity( 0.01       , // [m/s]
    //                         0.01       , // [m/s]
    //                         0.01       , // [m/s]
    //                         sp        );

    // Check position
    int land = 0;
    int setpoint = 1;
    int on_x_position = 0;
    int on_y_position = 0;
    int on_z_position = 0;
    int loop_cnt = 0;
    int i = 10;
    int land_delay = 14;
    while(1)
    {
        loop_cnt++;

        mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
        printf("Current Position = [ % .4f , % .4f , % .4f ]  , d=% .4f\n", pos.x, pos.y, pos.z, 
            distance(pos.x, pos.y, pos.z, sp.x, sp.y, sp.z));

#define DISTANCE_HAS_KNOWN
#undef DISTANCE_HAS_KNOWN

#ifdef DISTANCE_HAS_KNOWN
        if(api.get_setpoint_sendstatus() && setpoint){
            
            if(  (distance(pos.x, pos.y, pos.z, sp.x, sp.y, sp.z) <  0.3) ){

                api.set_setpoint_sendstatus(false);    // setpoint has been dealed

                printf("Arrival to setpoint, loiter here %2ds", i);
                while(i >= 0){
                    printf("\b\b\b%2ds", i);
                    fflush(stdout);
                    sleep(1);
                    i--;
                }
                printf("\n");

                i = 10;

                // if(setpoint){
                    
                    set_position( ip.x, ip.y+8, ip.z - 2.5, sp);
                    api.update_setpoint(sp);  // THEN pixhawk will try to move
                    setpoint--;
                // }
                sleep(1);
                land = 1;
            }
        }

        if( !(api.get_setpoint_sendstatus()) && (land == 1)){
            // printf("land...\n");
            // set_land(sp);
            // api.update_setpoint(sp); 
        }
#else
        i = 15;
        while(i >= 0){
            pos = api.current_messages.local_position_ned;
             printf("Current Position = [ % .4f , % .4f , % .4f ]  , d=% .4f\n", pos.x, pos.y, pos.z, 
                distance(pos.x, pos.y, pos.z, sp.x, sp.y, sp.z));
             
            printf("Arrival to setpoint, loiter here %2ds", i);
            printf("\b\b\b%2ds", i);
            printf("\n");
            // fflush(stdout);
            sleep(1);
            i--;
        }

        set_position( ip.x, ip.y+8, ip.z - 2.5, sp);
        api.update_setpoint(sp);  // THEN pixhawk will try to move

        i = 15;
        while(i >= 0){
            pos = api.current_messages.local_position_ned;
            printf("Current Position = [ % .4f , % .4f , % .4f ]  , d=% .4f\n", pos.x, pos.y, pos.z, 
                distance(pos.x, pos.y, pos.z, sp.x, sp.y, sp.z));
            printf("Arrival to setpoint, loiter here %2ds", i);
            printf("\b\b\b%2ds", i);
            printf("\n");
            // fflush(stdout);
            sleep(1);
            i--;
        }
        set_position( ip.x+10, ip.y+8, ip.z - 2.5, sp);
        api.update_setpoint(sp);  // THEN pixhawk will try to move
        printf("Misson done....\n");
       
        while(1){
            land_delay--;
            sleep(1);
            pos = api.current_messages.local_position_ned;
            printf("Current Position = [ % .4f , % .4f , % .4f ]  , d=% .4f\n", pos.x, pos.y, pos.z, 
                distance(pos.x, pos.y, pos.z, sp.x, sp.y, sp.z));

            if(!land_delay) {
                printf("land...\n");
                // set_land(sp);
                // api.update_setpoint(sp); 
                // api.toggle_land_control(true);
                api.toggle_return_control(true);
             }   

        }

#endif

        sleep(1);
    }

    printf("\n");

error:
    //   STOP OFFBOARD MODE
    api.disable_offboard_control();

    //   DISARMED
    api.vehicle_disarm();

    // now pixhawk isn't listening to setpoint commands

#ifdef DEBUG
    //   GET A MESSAGE

    // local position in ned frame
    mavlink_local_position_ned_t pos = messages.local_position_ned;
    printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
    printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

    // hires imu
    mavlink_highres_imu_t imu = messages.highres_imu;
    printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
    printf("    ap time:     %lu \n", imu.time_usec);
    printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
    printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
    printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
    printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
    printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
    printf("    temperature: %f C \n"       , imu.temperature );

    printf("\n");
#endif
    return;
}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate> -m <takeoff_mode>";
    static bool mode_enable = false;
    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n",commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                uart_name = argv[i + 1];

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                baudrate = atoi(argv[i + 1]);

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Takeoff Mode
        if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--mode") == 0) {

            if (argc > i + 1) {
                if( strcmp(argv[i+1], "auto") == 0){
                    takeoff_mode = TAKE_OFF_AUTOMATIC;
                    mode_enable = true;
                }
                else if( strcmp(argv[i+1], "manual") == 0){
                    takeoff_mode = TAKE_OFF_MANUAL_OR_GCS;
                    mode_enable = true;
                }

            } else {
                printf("%s\n",commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    if (!mode_enable){
        printf("error: <takeoff_mode> is needed: [ manual | auto ]\n\n");
        printf("%s\n\n",commandline_usage);
        throw EXIT_FAILURE;
    }

    // Done!
    return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
    printf("\n");
    printf("TERMINATING AT USER REQUEST\n");
    printf("\n");

    // autopilot interface
    try {
        autopilot_interface_quit->handle_quit(sig);
    }
    catch (int error){}

    // serial port
    try {
        serial_port_quit->handle_quit(sig);
    }
    catch (int error){}

    // end program here
    exit(0);

}


// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
    // This program uses throw, wrap one big try/catch here
    try
    {
        int result = top(argc,argv);
        return result;
    }

    catch ( int error )
    {
        fprintf(stderr,"mavlink_control threw exception %i \n" , error);
        return error;
    }

}