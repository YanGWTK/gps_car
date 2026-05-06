/*
 * mean.h
 *
 *  Created on: 2025年3月1日
 *      Author: 29500
 */

#ifndef SRC_USER_MEAN_H_
#define SRC_USER_MEAN_H_

typedef struct
{
    int current;
    int up;//向上翻索引号
    int down;//向下翻索引号
    int enter;//确认索引号
    void (*current_operation)();//当前页面的索引号要执行的显示函数，这是一个函数指针
} key_table;
extern key_table table_dispaly[30];
void Menu();//菜单函数

void show_num();
void servo_test();
void motor_test();
void  Menu_flash_clear(int PAGE_INDEX);
void fun_0();

///*********第1层***********/
    void fun_a1();


    void fun_b1() ;


    void fun_c1();


    void fun_d1();

    ///*********第1层***********/

    ///*********第2层***********/
    void fun_a21();


    void fun_a22();


    void fun_a23();


    void fun_a24();

    void fun_b21();


    void fun_b22();


    void fun_b23() ;

    void fun_b24();

    void fun_c21();


    void fun_c22();

    void fun_c23();

    void fun_c24();

    void fun_a31();  //GPS点位采集并存入对应FLASH

    void fun_a32();

    void fun_a33();//清楚GPS的FLASH

    void fun_b31();

    void fun_b32();

    void fun_b33();

    void fun_c31();

    void fun_c32();

    void fun_c33();
#endif /* SRC_USER_MEAN_H_ */
