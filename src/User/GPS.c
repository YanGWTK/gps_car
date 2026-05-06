/*
 * GPS.c
 *
 *  Created on: 2024年12月26日
 *      Author: 29500
 */
#include "include.h"
double point[100][2];//存点  lat为0 lon为1
GPS_param GPS_data;
GPS_param last_data;
uint32_t lat1;
uint32_t lat2;
uint32_t lon1;
uint32_t lon2;
double temp=2;//补偿gps


static double ref_lat = 0.0;
static double ref_lon = 0.0;
static double min_lat = DBL_MAX, max_lat = -DBL_MAX;
static double min_lon = DBL_MAX, max_lon = -DBL_MAX;
static const int SCREEN_WIDTH = 162;
static const int SCREEN_HEIGHT = 132;
#define PATH_COLOR U16_RED
#define POINT_COLOR U16_BLUE
#define BG_COLOR U16_WHITE
static PathBoundary path_bounds = {0};
static double auto_scale_factor = 1.0;
static const double MARGIN_RATIO = 0.1; // 边界留空比例
//以下为显示坐标在路线图上
void calculate_path_bounds() {
    if(max_num < 1) return;

    const double METERS_PER_DEGREE = 111319.9;
    path_bounds.min_east = path_bounds.max_east = 0;
    path_bounds.min_north = path_bounds.max_north = 0;

    // 以第一个点为参考原点
    double ref_lat = point[0][0];
    double ref_lon = point[0][1];

    // 遍历所有点计算相对位置
    for(int i=0; i<max_num+1; i++){
        double delta_lon = point[i][1] - ref_lon;
        double delta_lat = point[i][0] - ref_lat;

        double east = delta_lon * METERS_PER_DEGREE * cos(ref_lat * PI / 180);
        double north = delta_lat * METERS_PER_DEGREE;

        // 更新边界
        if(east < path_bounds.min_east) path_bounds.min_east = east;
        if(east > path_bounds.max_east) path_bounds.max_east = east;
        if(north < path_bounds.min_north) path_bounds.min_north = north;
        if(north > path_bounds.max_north) path_bounds.max_north = north;
    }

    // 计算实际范围（考虑留白）
    double east_range = (path_bounds.max_east - path_bounds.min_east) * (1 + MARGIN_RATIO*2);
    double north_range = (path_bounds.max_north - path_bounds.min_north) * (1 + MARGIN_RATIO*2);

    // 计算缩放比例
    double x_scale = SCREEN_WIDTH / (east_range ? east_range : 1); // 防止除零
    double y_scale = SCREEN_HEIGHT / (north_range ? north_range : 1);
    auto_scale_factor = fmin(x_scale, y_scale);
}

// 改进的坐标转换函数
ScreenPoint gps_to_screen(double lat, double lon) {
    const double METERS_PER_DEGREE = 111319.9;
    double ref_lat = point[0][0];
    double ref_lon = point[0][1];

    // 计算相对位置
    double delta_lon = lon - ref_lon;
    double delta_lat = lat - ref_lat;

    double east = delta_lon * METERS_PER_DEGREE * cos(ref_lat * PI / 180);
    double north = delta_lat * METERS_PER_DEGREE;

    // 应用动态缩放和居中
    double scaled_east = (east - path_bounds.min_east + (path_bounds.max_east - path_bounds.min_east)*MARGIN_RATIO) * auto_scale_factor;
    double scaled_north = (north - path_bounds.min_north + (path_bounds.max_north - path_bounds.min_north)*MARGIN_RATIO) * auto_scale_factor;

    // 转换为屏幕坐标（Y轴翻转）
    int screen_x = (int)scaled_east;
    int screen_y = SCREEN_HEIGHT - (int)scaled_north;

    // 边界保护
    screen_x = (screen_x < 0) ? 0 : (screen_x >= SCREEN_WIDTH) ? SCREEN_WIDTH-1 : screen_x;
    screen_y = (screen_y < 0) ? 0 : (screen_y >= SCREEN_HEIGHT) ? SCREEN_HEIGHT-1 : screen_y;

    return (ScreenPoint){screen_x, screen_y};
}

// 更新绘制函数
void draw_gps_path(void) {
    if(max_num < 1) return;

    // 计算动态比例
    calculate_path_bounds();

    // 绘制路径
    ScreenPoint prev = gps_to_screen(point[0][0], point[0][1]);
    for(int i=1; i<max_num+1; i++){
        ScreenPoint curr = gps_to_screen(point[i][0], point[i][1]);
        Display_DrawLine(prev.x, prev.y, curr.x, curr.y, PATH_COLOR);
        prev = curr;
    }

    // 绘制点标记（第一个点特别标注）
    for(int i=0; i<max_num+1; i++){
        ScreenPoint p = gps_to_screen(point[i][0], point[i][1]);
        if(i == 0) {
            Display_DrawCircle(p.x, p.y, 3, POINT_COLOR);
            Display_Fill(p.x-1, p.y-1, p.x+1, p.y+1, POINT_COLOR);
        } else {
            Display_DrawRectangle(p.x-1, p.y-1, p.x+1, p.y+1, POINT_COLOR);
        }
    }

    // 添加比例尺
    char scale_text[20];
    double actual_meters = 10 / auto_scale_factor; // 10像素对应的实际米数
}





//
void gps_Init()
{
    last_data.LAT=0;
    last_data.LON=0;
GPS_data.LAT_CORRECT=0;//纬度纠正
GPS_data.LON_CORRECT=0;//经度纠正
CCU6_InitConfig(CCU60,CCU6_Channel1,100);
}
void gps_get()
{
    parseGpsBuffer();
    BD_getdata(&point_p);
GPS_data.LAT=point_p.lat;
GPS_data.LON=point_p.lon;
if(GPS_data.LAT!=last_data.LAT||GPS_data.LON!=last_data.LON)
{point_p.Direction= get_two_points_azimuth(last_data.LAT, last_data.LON, GPS_data.LAT, GPS_data.LON);
GPS_data.angle=point_p.Direction;}
last_data.LAT=GPS_data.LAT;
last_data.LON=GPS_data.LON;

}

double get_angle(GPS_param *Point1, double lon,double lat)
{
    double angle=0.0;
    angle = atan2(Point1->LON - lon, Point1->LAT - lat);//(-180, 180)
    angle = angle*180/3.1415926+180;
    if(angle < 0)
        return 360+angle;
    else
        return angle;

}
double angle_weighting(double angle1,double angle2,double weight1,double weight2)//0-360坐标系的加权
{
  double X1,Y1,X2,Y2;
  X1=sin((angle1*PI)/180.0);
  Y1=cos((angle1*PI)/180.0);
  X2=sin((angle2*PI)/180.0);
  Y2=cos((angle2*PI)/180.0);
  double X_result,Y_result;
  X_result=X1*weight1+X2*weight2;
  Y_result=Y1*weight1+Y2*weight2;
  double angle_result=atan2(X_result,Y_result)*180/PI;
  if(angle_result<0) angle_result+=360;//转化为0-360坐标系
  return angle_result;
}

double get_two_points_distance (double latitude1, double longitude1, double latitude2, double longitude2)
{
    const double EARTH_RADIUS = 6378137;                                        // 地球半径(单位：m)
    double rad_latitude1 = 0;
    double rad_latitude2 = 0;
    double rad_longitude1 = 0;
    double rad_longitude2 = 0;
    double distance = 0;
    double a = 0;
    double b = 0;

    rad_latitude1 = ANGLE_TO_RAD(latitude1);                                    // 根据角度计算弧度
    rad_latitude2 = ANGLE_TO_RAD(latitude2);
    rad_longitude1 = ANGLE_TO_RAD(longitude1);
    rad_longitude2 = ANGLE_TO_RAD(longitude2);

    a = rad_latitude1 - rad_latitude2;
    b = rad_longitude1 - rad_longitude2;

    distance = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rad_latitude1) * cos(rad_latitude2) * pow(sin(b / 2), 2)));   // google maps 里面实现的算法
    distance = distance * EARTH_RADIUS;

    return distance;
}

double get_two_points_azimuth (double latitude1, double longitude1, double latitude2, double longitude2)
{
    latitude1 = ANGLE_TO_RAD(latitude1);
    latitude2 = ANGLE_TO_RAD(latitude2);
    longitude1 = ANGLE_TO_RAD(longitude1);
    longitude2 = ANGLE_TO_RAD(longitude2);

    double x = sin(longitude2 - longitude1) * cos(latitude2);
    double y = cos(latitude1) * sin(latitude2) - sin(latitude1) * cos(latitude2) * cos(longitude2 - longitude1);
    double angle = RAD_TO_ANGLE(atan2(x, y));
    return ((angle > 0) ? angle : (angle + 360));
}
