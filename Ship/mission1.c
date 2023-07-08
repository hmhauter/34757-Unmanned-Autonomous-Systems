#include <sched.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <linux/serial.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <expat.h>
#include <dlfcn.h>
#include <curses.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include <rhd.h>


#define LOG_FOLDER_PATH "/home/roc/src/rhd-mbzirc/testprogs/group-nine/dataLogs"
#define ACCELEROMETER_FILTER_ALPHA 0.25
#define GYROSCOPE_FILTER_ALPHA 0.15
#define DELTA_TIME 0.025

// WIP (comments to myself)
// 1543 is the magic threshold for PWM FORWARD
// find out why propellers are turned
// find out why cw cw config (same as big cat?)
// pool is  6.5x3.5x3m

typedef struct {
    float angle_setpoint;
    float speed_setpoint;
    float distance_setpoint;
} setpoint;

int calcAngle(int, int);
int constrainPWM(int);
int calcPWMThrustForce(float);
int calcPWMThrustFrontJoy(int);
int calcPWMThrustBackJoy(int);
int calcPWMThrustFrontForce(float);
int calcPWMThrustBackForce(float);
float lowPassFilter(float, float, float);
void mission(setpoint, bool, FILE*);

void end(int);
int setup();

int connectRhd(char *hostname);


int serial_port;
// yes, speed and angle (!!!) should be global 
float angle = 0.0;
float prev_angle = 0.0;
float logTime = 0.0;
// joy values
int usv_stop = 0;
int running = 1;

// int debug() {
//     char buffer[20];
//         while (running) {
//         rhdSync();

//         // read joy input
//         usv_stop = readValueNamed("joybuttons",1);

//         if(usv_stop == 1) {
//             printf("Security stop activated\n");
//             // stop loop
//             running = 0;
//             sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
//     	    serialPuts(serial_port, buffer);
//             return -1;
//         } 
//         if(logTime < 4) {
//           sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1600, 1600);
//         serialPuts(serial_port, buffer);  
//         } 
//         if(logTime >= 4) {
//             sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
//             serialPuts(serial_port, buffer);  

//         } 
//         if(logTime >= 7) {
//             sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
//             serialPuts(serial_port, buffer);  
//             running = 0;
//             break;
//         } 
//         }
// }

int main(int argc, char * argv[]) {
    const int SIZE = 11;
    setpoint setpointList[] = {
        {.angle_setpoint = 0.0, .speed_setpoint = 2.5, .distance_setpoint = 13.0},
        {.angle_setpoint = 180.0, .speed_setpoint = 0.0, .distance_setpoint = -1},
        {.angle_setpoint = 180.0, .speed_setpoint = 2.5, .distance_setpoint = 9.0},
        {.angle_setpoint = 0.0, .speed_setpoint = 0.0, .distance_setpoint = -1},
        {.angle_setpoint = 0.0, .speed_setpoint = 2.5, .distance_setpoint =8.5},
        {.angle_setpoint = 180.0, .speed_setpoint = 0.0, .distance_setpoint = -1},
        {.angle_setpoint = 180.0, .speed_setpoint = 2.5, .distance_setpoint = 8.0},
        {.angle_setpoint = 0.0, .speed_setpoint = 0.0, .distance_setpoint = -1},
        {.angle_setpoint = 0.0, .speed_setpoint = 2.5, .distance_setpoint = 8.5},
        {.angle_setpoint = 180.0, .speed_setpoint = 0.0, .distance_setpoint = -1},
        {.angle_setpoint = 180.0, .speed_setpoint = 2.5, .distance_setpoint = 8.5},
    };
      

    int hasError = setup();
    if(hasError == 1) {
        return 1;
    }

    // steup for logging
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char filename[1024];
    snprintf(filename, sizeof(filename), "%s/%04d%02d%02d_%02d%02d%02dAA.csv",
             LOG_FOLDER_PATH, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
             tm->tm_hour, tm->tm_min, tm->tm_sec);
    FILE *logFile = fopen(filename, "w");
    if (logFile == NULL) {
        printf("Failed to open log file.\n");
        return 1;
    }

    // set initial pose
    char buffer[20];
    sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
    serialPuts(serial_port, buffer);

    // run mission for setpoints
    for(int i = 0; i < SIZE; i++) {
        mission(setpointList[i], 1, logFile);
    }
    printf("I am done... Good Bye \n");
    sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
    serialPuts(serial_port, buffer);
    fclose(logFile);
    serialClose(serial_port);
    
  
}

void mission(setpoint setpoint, bool hasLogging, FILE* logFile) {
    printf("---------------GET NEW SETPOINT---------------------");
   
    running = 1;
    char buffer[20];
    
    //values from IMU
    float prev_Acc_y = 0.0, Acc_y_filtered;
    float prev_Gyro_z = 0.0, Gyro_z_filtered;
    float Acc_y=0.0, Gyro_z=0.0;

    // initial values
    float thrust_left = 0.0, thrust_right = 0.0;
    int pwm_left = 1500, pwm_right = 1500;
    int left_angle = 90, right_angle = 90;
    float thrust_sum=0.0, thrust_diff=0.0;

    // speed control
    float speed = 0.0;
    float prev_speed = 0.0;

    // distance tracking 
    float prev_distance = 0.0, distance = 0.0;

    // controller values
    // float Kp_theta = 0.5, Kp_thetadot = 2.0, Kp_speed = 1.5;
    float Kp_theta = 1.0, Kp_thetadot = 0.2, Kp_speed = 1.5;
    float ang_setpoint = setpoint.angle_setpoint;
    float speed_setpoint = setpoint.speed_setpoint; 
    float distance_setpoint = setpoint.distance_setpoint;

    float go_time = 0.0;
    
    


    while (running) {
        rhdSync();

        // read joy input
        usv_stop = readValueNamed("joybuttons",1);

        Gyro_z = (float)(readValueNamed("GyroZ",0)/1000);
        Gyro_z_filtered = lowPassFilter(prev_Gyro_z, Gyro_z, GYROSCOPE_FILTER_ALPHA);
        prev_Gyro_z = Gyro_z_filtered;
        Acc_y = (float)(readValueNamed("AccY",0)/100.0); 
        Acc_y_filtered = lowPassFilter(prev_Acc_y, Acc_y, ACCELEROMETER_FILTER_ALPHA);
        prev_Acc_y = Acc_y_filtered;
        

    	if(usv_stop == 1) {
            printf("Security stop activated\n");
            // stop loop
            running = 0;
            sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
    	    serialPuts(serial_port, buffer);
            return;
        } 
         
            // change to filter
        	// printf("The angular velocity in the Z-direction is: %lf\n", Gyro_z_filtered);
            // printf("The linear velocity in the Y-direction is: %lf\n", Acc_y_filtered);
            // integration loop for control system
            // ANGLE
            angle = prev_angle + Gyro_z_filtered*DELTA_TIME;
            thrust_diff = ((ang_setpoint - angle)*Kp_theta - Gyro_z_filtered)*Kp_thetadot;
            // printf("Angle: %f \n", angle);
            // printf("Angle Error: %f \n", (ang_setpoint - angle));
            // printf("Thrust Diff: %f \n", thrust_diff);
            // distribute thrust
            // if thrust > 0 right should be actuated forward and left backward
            // if thrust < 0 left should be actuated forward and right backward
            thrust_left = -thrust_diff / 2.0;
            thrust_right = thrust_diff / 2.0;

            // printf("Thrust LEFT: %f \n", thrust_left);
            // printf("Thrust RIGHT: %f \n", thrust_right);

            
            // SPEED AND TRAVEL DISTANCE
            speed = prev_speed + Acc_y_filtered * DELTA_TIME;
            distance = prev_distance + speed * DELTA_TIME + 0.5 * Acc_y_filtered * DELTA_TIME * DELTA_TIME;
            if(speed_setpoint == 0) {
                thrust_sum = 0.0;
            } else {
                thrust_sum = (speed_setpoint - speed)*Kp_speed;
            }
            

            // printf("Speed: %f \n", speed);
            // printf("Travel Distance: %f \n", distance);
            // printf("Speed Error: %f \n", (speed_setpoint - speed));
            // printf("Thrust Sum: %f \n", thrust_sum);

            // for now only do forward 
            thrust_left += thrust_sum / 2;
            thrust_right += thrust_sum / 2;

            // define it for now - may change later
            left_angle = 90;
            right_angle = 90;

            pwm_left = calcPWMThrustForce(0.97 * thrust_left);
            pwm_right = calcPWMThrustForce(1.03 * thrust_right); 
            pwm_left = constrainPWM(pwm_left);
            pwm_right = constrainPWM(pwm_right);

            // printf("angle left: %d, angle right: %d\n", left_angle, right_angle);
            // printf("thrust left: %d, thrust right: %d\n", pwm_left, pwm_right);
            sprintf(buffer, "%d,%d,%d,%d\n", left_angle, right_angle, pwm_left, pwm_right);
            serialPuts(serial_port, buffer);
            // set values for next integration step
            prev_angle = angle;
            prev_speed = speed;
            prev_distance = distance;
            printf("TIME POINT: %f\n", go_time);
            if(distance_setpoint == -1) {
                // only check for angle
                // int ang_thr = 1;
                // if(abs(angle) >= abs(ang_setpoint + ang_thr) || abs(angle) <= abs(ang_setpoint - ang_thr)) {
                if(abs((ang_setpoint - angle)) <= 1.5) {
                    printf("ANGLE Goal reached: %.6f\n", angle);
                    // stop loop
                    running = 0;
                    // sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
                    // serialPuts(serial_port, buffer);
              

                }
            } else if(go_time >= distance_setpoint) {
                printf("DISTANCE Goal reached: %.6f\n", distance);
                // stop loop
                running = 0;
                // sprintf(buffer, "%d,%d,%d,%d\n", 90, 90, 1500, 1500);
                // serialPuts(serial_port, buffer);
            }
            if(hasLogging == 1) {
                fprintf(logFile, "%.6f,%.6f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n", logTime, Gyro_z, angle, Acc_y, Acc_y_filtered, speed, distance, thrust_left, thrust_right, thrust_diff, pwm_left, pwm_right);   
                fflush(logFile);
            }             
        logTime += DELTA_TIME;
        go_time += DELTA_TIME;
    }

}

float lowPassFilter(float previousValue, float currentValue, float filter_alpha) {
    return (previousValue * (1 - filter_alpha)) + (currentValue * filter_alpha);
}

int constrainPWM(int pwm) {
    if(pwm < 1250) {
        pwm = 1250;
    } else if (pwm > 1750) {
        pwm = 1750;
    }
    return pwm;
}

int connectRhd(char *hostname) {

  if (rhdConnect('w',hostname,DEFAULTPORT) <= 0) {
      printf("Error: Failed to connect to RHD on host: %s\n",hostname);
      return -1;
   } else {
      printf("Successfully connected to RHD\n");
   }
  return 1;

}

void end (int sig) {
    running = 0;
    printf("Shutting down usr client!!!!!!!\n");
    exit(1);
}

int setup() {

    signal(SIGTERM, end);
    signal(SIGINT, end);
    signal(SIGKILL, end);

    if ((serial_port = serialOpen ("/dev/ttyAMA0", 38400)) < 0)   /* open serial port */
    {
	fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    	return 1 ;
    }

    if (wiringPiSetup () == -1)                                   /* initializes wiringPi setup */
    {
    	fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    	return 1 ;
    }

    if (connectRhd("127.0.0.1") < 0) {
      exit(0);
    }
    return 0;
}


int calcAngle(int joystick_val, int current_angle){
    // deviding joy_stick potentiometer value by 3500 as max is 32767
    current_angle += (joystick_val/3500);
    if(current_angle > 180){
        current_angle = 180;
    }else if(current_angle < 0){
        current_angle = 0;
    }
    return current_angle;
}

int calcPWMThrustForce(float thrust_val) {
    if (thrust_val > 0) {
        return calcPWMThrustFrontForce(thrust_val);
    } 
    return calcPWMThrustBackForce(abs(thrust_val));
}

int calcPWMThrustFrontForce(float thrust_val){
    if(thrust_val == 0.0) {
        return 1500;
    }
    // convert N to kgf
    float thrust_kgf = thrust_val / 9.81;
    // use linear approximation of thrust - pwm relation to get PWM
    int pwm = (int)((thrust_kgf + 13.26) / 0.0085);
    if (pwm <= 1543) {
        return 1500;
    }
    return pwm;
}

int calcPWMThrustBackForce(float thrust_val){
    if(thrust_val == 0.0) {
        return 1500;
    }
    // convert N to kgf
    float thrust_kgf = thrust_val / 9.81;
    // use linear approximation of thrust - pwm relation to get PWM
    return (int)((thrust_kgf -9.55) / (-0.0067));
}

int calcPWMThrustFrontJoy(int thrust_val){
    return (thrust_val - (-32767)) * (1900 - 1500) / (32767 - (-32767)) + 1500;
}

int calcPWMThrustBackJoy(int thrust_val){
    return (thrust_val - (-32767)) * (1100 - 1500) / (32767 - (-32767)) + 1500;
}
