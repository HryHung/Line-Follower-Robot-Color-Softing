#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define N_SENSOR 5
#define LOOP 20
#define BASE_RPM 200
#define RANGE 30
#define Kp 0.05   // hệ số tạm (proportional)

// ---- Base pattern: line ở giữa ----
int base_pattern[N_SENSOR] = {200, 500, 800, 500, 200};

// ---- Sinh dữ liệu sensor ----
void generate_sensor_data(int sensor[]) {
    for (int i = 0; i < N_SENSOR; i++) {
        int noise = (rand() % (2*RANGE+1)) - RANGE;
        sensor[i] = base_pattern[i] + noise;
    }
}

// ---- Tính error centroid ----
float compute_error(int sensor[]) {
    int weight[N_SENSOR] = {-2, -1, 0, 1, 2}; // vị trí
    long sum_w = 0, sum_s = 0;
    for (int i = 0; i < N_SENSOR; i++) {
        sum_w += (long)weight[i] * sensor[i];
        sum_s += sensor[i];
    }
    if (sum_s == 0) return 0; // tránh chia 0
    return (float)sum_w / sum_s; // error: âm = lệch trái, dương = lệch phải
}

// ---- Điều khiển motor dựa vào error ----
void drive_motor(int sensor[], int *left, int *right) {
    float error = compute_error(sensor);
    int correction = (int)(Kp * error * BASE_RPM);

    *left  = BASE_RPM - correction;
    *right = BASE_RPM + correction;
}

// ---- Hiển thị ----
void display_status(int sensor[], int left, int right) {
    printf("Line_sensor: ");
    for (int i = 0; i < N_SENSOR; i++) {
        printf("%4d ", sensor[i]);
    }
    printf(" || RPM (left/right): %3d %3d\n", left, right);
}

// ---- MAIN ----
int main() {
    int sensor[N_SENSOR];
    int rpm_left, rpm_right;

    srand(time(NULL));

    for (int t = 0; t < LOOP; t++) {
        generate_sensor_data(sensor);
        drive_motor(sensor, &rpm_left, &rpm_right); // giờ phụ thuộc sensor
        display_status(sensor, rpm_left, rpm_right);
    }

    return 0;
}
