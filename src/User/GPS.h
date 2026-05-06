/*
 * GPS.h
 *
 *  Created on: 2024年12月26日
 *      Author: 29500
 */

#ifndef SRC_USER_GPS_H_
#define SRC_USER_GPS_H_
#define ecoder_min 60 //开始融合的最低速度
#define sate_min 5   //开始融合的最小卫星数量
extern double point[100][2];
extern uint32_t lat1;
extern uint32_t lat2;
extern uint32_t lon1;
extern uint32_t lon2;
extern double temp;
typedef struct
{
  double angle;
  double LAT;//纬度
  double LON;//经度
  double LAT_CORRECT;//纬度纠正系数
  double LON_CORRECT;//经度修正系数
}GPS_param;

//
void draw_gps_path(void);
void display_path_screen(void);
typedef struct {
    int x;
    int y;
} ScreenPoint;
typedef struct {
    double x;  // 东向距离（米）
    double y;  // 北向距离（米）
} LocalCoord;
typedef struct {
    double min_east;
    double max_east;
    double min_north;
    double max_north;
} PathBoundary;
void set_rotation_angle(double angle);
void set_reference_point(double lat, double lon);
ScreenPoint gps_to_screen(double lat, double lon);
void calculate_screen_boundaries(void);
//
void GPS_To_Local(double lat, double lon, LocalCoord* local);

void Local_To_Screen(LocalCoord* local, int* screen_x, int* screen_y);

void Draw_Direction_Arrow(int x1, int y1, int x2, int y2, unsigned short color);

void Draw_Full_Path();
void GPS_To_Screen(double lat, double lon, int* screen_x, int* screen_y);
void Draw_GPS_Path();
extern GPS_param GPS_data;
extern GPS_param last_data;

void gps_Init();

void gps_get();

double get_angle();
double angle_weighting(double angle1,double angle2,double weight1,double weight2);
double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2);
double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2);
#endif /* SRC_USER_GPS_H_ */
