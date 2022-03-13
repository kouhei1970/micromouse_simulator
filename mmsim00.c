#include <stdio.h>
#include <cairo/cairo.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#define MAZE_SIZE (16)
#define SECTION (180)
#define GOAL_X (7)
#define GOAL_Y (7)

void make_mapdata(void);
void reset_map(void);
void round_dir(char d);
char update_mouse_position(char dir);
char get_map(char x, char y, char dir);
void set_map(char x, char y, char dir, char state);
void draw_v_wall(int x, int y);
void draw_h_wall(int x, int y);
void draw_mouse(double x, double y, double angle);
void erase_mouse(double x, double y, double angle);
void draw_maze(int width, int height);
double x_dot_2(double x, double t, double *value);
double y_dot_2(double y, double t, double *value);
double psi_dot_1(double psi, double t, double *value);
double v_dot_1(double v, double t, double *value);
double r_rate_dot_1(double r_rate, double t, double *value);
double rk4(double (*dxdt)(double, double, double*), double x, double t, double h, int n, ...);
void one_step(int dir, double x, double y, double angle, double velocity, double framerate);
void r_turn(double x, double y, double angle, double velocity, double framerate);
void l_turn(double x, double y, double angle, double velocity, double framerate);
void straight(void);
void right_turn(void);
void left_turn(void);
void turn180(void);
void mode0(void);
void calc_trajectry(void);

//2019+alpha Clasic mouse expart final maze
char maze[33][66]={
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",//    0
    "|                                                               |",//15  1
    "+   +---+---+---+---+---+---+---+---+---+---+---+---+---+   +---+",//    2
    "|                               |       |       |   |           |",//14  3
    "+---+---+---+---+---+---+---+   +   +   +   +   +   +   +   +   +",//    4
    "|               |       |       |   |       |           |   |   |",//13  5
    "+   +---+---+   +   +   +   +   +   +---+   +---+   +---+---+   +",//    6
    "|   |       |       |       |   |       |               |       |",//12  7
    "+   +   +   +---+---+---+---+   +---+   +---+---+---+---+   +---+",//    8
    "|   |   |   |   |   |   |       |       |       |       |       |",//11  9
    "+   +   +   +   +   +   +   +---+   +---+   +   +   +   +---+   +",//   10
    "|   |   |                   |       |       |       |       |   |",//10 11
    "+   +   +---+   +   +   +---+   +---+   +---+---+---+---+   +   +",//   12
    "|   |       |   |   |   |       |           |       |           |",// 9 13
    "+   +---+   +---+---+---+   +---+---+   +---+   +   +   +---+---+",//   14
    "|                   |                   |       |   |           |",// 8 15
    "+---+   +---+   +---+---+   +   +   +---+   +   +   +---+---+   +",//   16
    "|                       |   |       |       |   |               |",// 7 17
    "+   +---+   +---+   +   +   +---+---+   +---+---+---+---+---+---+",//   18
    "|                   |   |   |   |   |                           |",// 6 19
    "+---+   +---+   +   +   +---+   +   +---+---+---+---+---+---+   +",//   20
    "|   |           |           |                   |       |       |",// 5 21
    "+   +---+   +   +   +---+   +   +---+   +   +   +   +   +   +---+",//   22
    "|   |       |       |       |   |       |   |       |           |",// 4 23
    "+   +   +   +   +---+   +---+   +   +---+   +---+---+---+---+---+",//   24
    "|       |       |       |       |   |           |       |       |",// 3 25
    "+   +   +   +---+   +---+   +---+   +---+   +   +   +   +   +   +",//   26
    "|   |       |       |       |       |       |       |       |   |",// 2 27
    "+   +   +---+   +---+   +---+   +   +   +   +   +   +   +   +   +",//   28
    "|           |                   |       |       |       |       |",// 1 29
    "+   +   +   +---+---+---+---+---+---+---+---+---+---+---+---+   +",//   30
    "|   |   |                                                       |",// 0 31
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"//    32
    // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
};

unsigned char mapdata[16][16];
unsigned char map[16][16];
unsigned char cmap[16][16];
unsigned char smap[16][16];
char mouse_x=0;
char mouse_y=0;
char mouse_dir=0;
short Index=0;

double Mx=90.0;
double My=90.0;
double Mangle=0.0;
double Mv =600.0;
double Momega=1.5*M_PI;
int Framerate=30;
char folder[]="imgx";

cairo_surface_t *CS;
cairo_t *C;

void output_to_img(double x, double y, double angle)
{
  char str[50];
  sprintf(str,"%s/test%05d.png",folder, Index);
  draw_mouse(x, y, angle);
  cairo_surface_write_to_png( CS, str );
  erase_mouse(x, y, angle);
  Index++;
}

void calc_trajectry(void)
{
  double x = (180*16+12)/2;
  double y = 180*2+90.0;
  double angle = 0.0;
  double v = 3000.0;
  double omega = M_PI;
  double t=0.0;
  double h=0.001;
  double fin=2.0;
  double framerate=60.0;
  double sample_time = 0.0;

  cairo_set_source_rgb( C, 0, 0, 0 );
  cairo_rectangle( C, 0, 0, 180*16+12, 180*16+12 );
  cairo_fill( C );

  while(t<fin)
  {
    if(t>=sample_time){
      printf("%f %f\n", x, y);
      sample_time = sample_time + 1/framerate;
      output_to_img(x, y, angle);
    }
    angle = angle + omega*h;
    x=rk4(x_dot_2, x, t, h, 2, v, angle);
    y=rk4(y_dot_2, y, t, h, 2, v, angle);
    t=t+h;
  }
  
}


void make_mapdata(void)
{
    unsigned char x,y;
    unsigned char n_wall,e_wall,s_wall,w_wall;

    for (y=0;y<16;y++){//南北方向
        for (x=0;x<16; x++) {//東西方向
            n_wall = 0x01*(maze[30-2*y][2+4*x]=='-');
            e_wall = 0x02*(maze[31-2*y][4+4*x]=='|');
            s_wall = 0x04*(maze[32-2*y][2+4*x]=='-');
            w_wall = 0x08*(maze[31-2*y][  4*x]=='|');
            mapdata[x][y] = s_wall | w_wall | e_wall | n_wall;
        }
    }
  //printf("%02X\n",mapdata[1][0]);
}

//map配列をリセット
//迷路外周とスタート区画の壁情報は既知なので同時にセットする
void reset_map(void)
{
  mouse_x = 0;
  mouse_y = 0;
  mouse_dir = 0;

  for(short x = 0; x < MAZE_SIZE; x++){
    for(short y = 0; y < MAZE_SIZE; y++){
      smap[x][y]=0;
      cmap[x][y]=255;
      map[x][y] = 0x00;
      if((x == 0)&&(y == 0)) map[x][y]  = 0xfe;
      if((x == 1)&&(y == 0)) map[x][y]  = 0xcc;
      if((x == 0)&&(y == 1)) map[x][y]  = 0xc8;

      if(y == (MAZE_SIZE-1)) map[x][y] |= 0x11;
      if(x == (MAZE_SIZE-1)) map[x][y] |= 0x22;
      if(y == 0)             map[x][y] |= 0x44;
      if(x == 0)             map[x][y] |= 0x88;
    }
  }
}

//向きを変更
//正:時計回り 負:反時計回り
void round_dir(char d)
{
  mouse_dir += d;
  mouse_dir &= 0x03;
}


//座標更新
//dir		現在の方角
//return値	-1:エラー 1:正常
char update_mouse_position(char dir)
{
  if(dir < 0 || dir > 3){return -1;}

  const char dx[4] = {0,1,0,-1};
  const char dy[4] = {1,0,-1,0};

  char temp_x = mouse_x + dx[dir];
  char temp_y = mouse_y + dy[dir];

  if(temp_x < 0 || temp_y < 0 || temp_x >= MAZE_SIZE || temp_y >= MAZE_SIZE){
    return -1;
  }
  else{
    mouse_x = temp_x;
    mouse_y = temp_y;
    return 1;
  }
}

//壁情報をmap配列から取得
//x x座標
//y y座標
//dir 向き 0:北 1:東 2:南 3:西
//return値 1:エラー 0:壁なし 1:壁あり 2:未探索
char get_map(char x, char y, char dir)
{
  if(x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE || dir < 0 || dir > 3){
    return -1;
  }

  //未探索
  if(((map[x][y] >> dir)&0x10) == 0){
    return 2;
  }
  else{
    return ((map[x][y] >> dir)&0x01);
  }
}

//壁情報をmap配列にセット
//x		x座標
//y		y座標
//dir	向き 0:北 1:東 2:南 3:西
//state	壁の有無 0:壁なし 1:壁あり
void set_map(char x, char y, char dir, char state)
{
  if(x < 0 || y < 0 || x >= MAZE_SIZE || y >= MAZE_SIZE || dir < 0 || dir > 3){
    return;
  }

  if(state == 1){
    switch(dir){
      case 0:map[x][y]|=0x11; if(y<MAZE_SIZE-1){map[x][y+1]|=0x44;}break;
      case 1:map[x][y]|=0x22; if(x<MAZE_SIZE-1){map[x+1][y]|=0x88;}break;
      case 2:map[x][y]|=0x44; if(y>0)          {map[x][y-1]|=0x11;}break;
      case 3:map[x][y]|=0x88; if(x>0)          {map[x-1][y]|=0x22;}break;
    }
  }
  else if(state == 0){
    switch(dir){
      case 0:map[x][y]|=0x10;map[x][y]&=0xfe; if(y<MAZE_SIZE-1){map[x][y+1]|=0x40;map[x][y+1]&=0xfb;}break;
      case 1:map[x][y]|=0x20;map[x][y]&=0xfd; if(x<MAZE_SIZE-1){map[x+1][y]|=0x80;map[x+1][y]&=0xf7;}break;
      case 2:map[x][y]|=0x40;map[x][y]&=0xfb; if(y>0)          {map[x][y-1]|=0x10;map[x][y-1]&=0xfe;}break;
      case 3:map[x][y]|=0x80;map[x][y]&=0xf7; if(x>0)          {map[x-1][y]|=0x20;map[x-1][y]&=0xfd;}break;
    }
  }
}

void draw_v_wall(int x, int y)
{
  cairo_set_source_rgb( C, 1, 0, 0 );
  cairo_rectangle( C, x, y, 12, 180-16);
  cairo_fill( C );

}

//水平の壁を描画
void draw_h_wall(int x, int y)
{
  cairo_set_source_rgb( C, 1, 0, 0 );
  cairo_rectangle( C, x, y, 180-16, 12);
  cairo_fill( C );

}

void draw_mouse(double x, double y, double angle)
{
  double x1 = -35.0;
  double y1 = -35.0;
  double x2 =  35.0;
  double y2 = -35.0;
  double x3 =  35.0;
  double y3 =  35.0;
  double x4 =   0.0;
  double y4 =  55.0;
  double x5 = -35.0;
  double y5 =  35.0; 

  angle = angle-M_PI/2;
  double x1_=x1*cos(angle) - y1*sin(angle) + x;
  double y1_=x1*sin(angle) + y1*cos(angle) + y;
  double x2_=x2*cos(angle) - y2*sin(angle) + x;
  double y2_=x2*sin(angle) + y2*cos(angle) + y;
  double x3_=x3*cos(angle) - y3*sin(angle) + x;
  double y3_=x3*sin(angle) + y3*cos(angle) + y;
  double x4_=x4*cos(angle) - y4*sin(angle) + x;
  double y4_=x4*sin(angle) + y4*cos(angle) + y;
  double x5_=x5*cos(angle) - y5*sin(angle) + x;
  double y5_=x5*sin(angle) + y5*cos(angle) + y;

  cairo_set_source_rgb( C, 0, 1, 0 );
  cairo_set_line_width( C, 1);
  cairo_move_to( C, (int)(x1_+6), (int)(180*16+12-y1_-6) );
  cairo_line_to( C, (int)(x2_+6), (int)(180*16+12-y2_-6) );
  cairo_line_to( C, (int)(x3_+6), (int)(180*16+12-y3_-6) );
  cairo_line_to( C, (int)(x4_+6), (int)(180*16+12-y4_-6) );
  cairo_line_to( C, (int)(x5_+6), (int)(180*16+12-y5_-6) );
  cairo_close_path( C );
  cairo_fill(C);
  cairo_stroke( C );
}

void erase_mouse(double x, double y, double angle)
{
  //左上端の座標算出
  x = x - 80;
  y = y + 80;
  //画像座標系へ変換
  x = x + 6;
  y= 180*16+6 - y;
  cairo_set_source_rgb( C, 0, 0, 0 );
  cairo_rectangle( C, x, y, 160, 160);
  cairo_fill( C );

}

void draw_maze(int width, int height)
{
  /* background */

  cairo_set_source_rgb( C, 0, 0, 0 );
  cairo_rectangle( C, 0, 0, width, height );
  cairo_fill( C );
 
  /* red rectangle */
  for (int y=0;y<16;y++){//南北方向
    for (int x=0;x<16; x++) {//東西方向
      if(maze[2*y][2+4*x]=='-') draw_h_wall(x*180+14, y*180);
      if(maze[1+2*y][4*x]=='|') draw_v_wall(x*180, y*180+14);
      if(maze[2+2*y][2+4*x]=='-') draw_h_wall(x*180+14, (y+1)*180);
      if(maze[1+2*y][4+4*x]=='|') draw_v_wall((x+1)*180, y*180+14);
    }
  }

  for (int y=0;y<(180*17+12);y+=180)
  {
    for (int x=0;x<(180*17+12);x+=180)
    {
      cairo_set_source_rgb( C, 1, 0, 0 );
      cairo_rectangle( C, x, y, 12, 12);
      cairo_fill( C );
    }
  }
  //draw_mouse(c, 90+6, 180*15+90+6);

}

//value1: velocity
//value2: angle
double x_dot_2(double x, double t, double *value)
{
  double v=value[0];
  double psi=value[1];

  return v*cos(psi);
}

//value1: velocity
//value2: angle
double y_dot_2(double y, double t, double *value)//double v, double psi)
{
  double v=value[0];
  double psi=value[1];
  return v*sin(psi);
}

double psi_dot_1(double psi, double t, double *value)//double r_rate)
{
  double r_rate=value[0];
  return r_rate;
}

double v_dot_1(double v, double t, double *value)//double acc)
{
  double acc=value[0];
  return acc;
}

double r_rate_dot_1(double r_rate, double t, double *value)//double acc_angle)
{
  double acc_angle=value[0];
  return acc_angle;
}

double rk4(double (*dxdt)(double, double, double*), double x, double t, double h, int n, ...)
{
  va_list args;
  double *value;
  double k1,k2,k3,k4;

  value=(double*)malloc(sizeof(double) * n);
  va_start(args , n);
  for(int i=0;i<n;i++)
  {
    value[i]=va_arg(args, double);
  }
  va_end(args);
  
  k1 = h * dxdt(x, t, value);
  k2 = h * dxdt(x+0.5*h*k1, t+0.5*h, value);
  k3 = h * dxdt(x+0.5*h*k2, t+0.5*h, value);
  k4 = h * dxdt(x+h*k3, t+h, value);

  free(value);
  
  return x+(k1 + k2*2.0 + k3*2.0 + k4)/6;
}


void one_step(int dir, double x, double y, double angle, double velocity, double framerate)
{
  double fin_time=SECTION/velocity;
  int frame = (int)(fin_time*framerate);
  char str[100];
  double x0=x;
  double y0=y;

  if (dir==0)
  {
    while(y<(y0+SECTION))
    {
      sprintf(str,"%s/test%05d.png",folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png( CS, str );
      erase_mouse(x, y, angle);
      y = y + velocity/framerate;
      Index++;
    }
    sprintf(str,"%s/test%05d.png",folder, Index);
    draw_mouse(x, y0+SECTION, angle);
    cairo_surface_write_to_png( CS, str );
    erase_mouse(x, y0+SECTION, angle);
  }
  else if(dir==1)
  {
    while(x<(x0+SECTION))
    {
      sprintf(str,"%s/test%05d.png",folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png( CS, str );
      erase_mouse(x, y, angle);
      x = x + velocity/framerate;
      Index++;
    }
    sprintf(str,"%s/test%05d.png",folder, Index);
    draw_mouse(x0+SECTION, y, angle);
    cairo_surface_write_to_png( CS, str );
    erase_mouse(x0+SECTION, y, angle);
  }
  else if(dir==2)
  {
    while(y>(y0-SECTION))
    {
      sprintf(str,"%s/test%05d.png",folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png( CS, str );
      erase_mouse(x, y, angle);
      y = y - velocity/framerate;
      Index++;
    }
    sprintf(str,"%s/test%05d.png",folder, Index);
    draw_mouse(x, y0-SECTION, angle);
    cairo_surface_write_to_png( CS, str );
    erase_mouse(x, y0-SECTION, angle);
  }
  else if(dir==3)
  {
    while(x>(x0-SECTION))
    {
      sprintf(str,"%s/test%05d.png",folder, Index);
      draw_mouse(x, y, angle);
      cairo_surface_write_to_png( CS, str );
      erase_mouse(x, y, angle);
      x = x - velocity/framerate;
      Index++;
    }
    sprintf(str,"%s/test%05d.png",folder, Index);
    draw_mouse(x0-SECTION, y, angle);
    cairo_surface_write_to_png( CS, str );
    erase_mouse(x0-SECTION, y, angle);
  }
}

void r_turn(double x, double y, double angle, double velocity, double framerate)
{
  double fin_time=(M_PI/2)/velocity;
  int frame = (int)(fin_time*framerate);
  char str[100];
  double angle0=angle;

  while(angle<(angle0+M_PI/2))
  {
    sprintf(str,"%s/test%05d.png",folder, Index);
    draw_mouse(x, y, angle);
    cairo_surface_write_to_png( CS, str );
    erase_mouse(x, y, angle);
    angle = angle + velocity/framerate;
    Index++;
  }
  sprintf(str,"%s/test%05d.png",folder, Index);
  draw_mouse(x, y, angle0+(M_PI/2) );
  cairo_surface_write_to_png( CS, str );
  erase_mouse(x, y, angle0+(M_PI/2) );
}

void l_turn(double x, double y, double angle, double velocity, double framerate)
{
  double fin_time=(M_PI/2)/velocity;
  int frame = (int)(fin_time*framerate);
  char str[100];
  double angle0=angle;

  while(angle>(angle0-M_PI/2))
  {
    sprintf(str,"%s/test%05d.png",folder, Index);
    draw_mouse(x, y, angle);
    cairo_surface_write_to_png( CS, str );
    sprintf(str,"%s/test%05d.png",folder, Index);
    erase_mouse(x, y, angle);
    angle = angle - velocity/framerate;
    Index++;
  }
  sprintf(str,"%s/test%05d.png",folder, Index);
  draw_mouse(x, y, angle0-(M_PI/2) );
  cairo_surface_write_to_png( CS, str );
  erase_mouse(x, y, angle0-(M_PI/2) );
}

void straight(void)
{
  one_step(mouse_dir, Mx, My, Mangle, Mv, Framerate);
  if(mouse_dir==0)
  {
    My=My+(double)SECTION;
  }
  else if(mouse_dir==1)
  {
    Mx=Mx+(double)SECTION;
  }
  else if(mouse_dir==2)
  {
    My=My-(double)SECTION;
  }
  else if(mouse_dir==3)
  {
    Mx=Mx-(double)SECTION;
  }
}

void right_turn(void)
{
  r_turn(Mx, My, Mangle, Momega, Framerate);
  Mangle = Mangle + M_PI/2;
  Mangle = fmod(Mangle, 2*M_PI);
}

void left_turn(void)
{
  l_turn(Mx, My, Mangle, Momega, Framerate);
  Mangle = Mangle - M_PI/2;
  Mangle = fmod(Mangle, 2*M_PI);
}

void turn180(void)
{
  right_turn();
  right_turn();
}

//拡張左手法
void mode0(void)
{
  make_mapdata();
  reset_map();

  //Start
  straight();
  
  while(1){
    smap[mouse_x][mouse_y]=1;
    //座標更新（柱と柱のあいだで更新）
    update_mouse_position(mouse_dir);
    //マップ更新
    if(smap[mouse_x][mouse_y]==0)
    {
      //正面
      if(get_map(mouse_x, mouse_y, mouse_dir)== 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y]>>mouse_dir)&0x01 );
      }
      else
      {	//仮想壁挿入
        set_map(mouse_x, mouse_y, mouse_dir, 1);
      }

      //右
      if(get_map(mouse_x, mouse_y, (mouse_dir+1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir+1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir+1)&3) )&0x01 );
      }
      else
      {	//仮想壁挿入
        set_map(mouse_x, mouse_y, (mouse_dir+1)&3, 1);
      }

      //左
      if(get_map(mouse_x, mouse_y, (mouse_dir-1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir-1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir-1)&3) )&0x01);
      }
      else
      {	//仮想壁挿入
        set_map(mouse_x, mouse_y, (mouse_dir-1)&3, 1);
      }

    }
    else
    {
      //正面
      if(get_map(mouse_x, mouse_y, mouse_dir)== 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y]>>mouse_dir)&0x01 );
      }
      //右
      if(get_map(mouse_x, mouse_y, (mouse_dir+1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir+1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir+1)&3) )&0x01 );
      }
      //左
      if(get_map(mouse_x, mouse_y, (mouse_dir-1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir-1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir-1)&3) )&0x01 );
      }
    }

    printf("(%7.3f, %2d, %2d, %02X, %02X)\n", 
        (double)Index/Framerate, mouse_x, mouse_y, mapdata[mouse_x][mouse_y], map[mouse_x][mouse_y]);
    fflush(stdout);
    //スタートに戻ったら終了
    if (mouse_x==GOAL_X && mouse_y==GOAL_Y)break;

    //行動
    if(get_map(mouse_x, mouse_y, (mouse_dir-1)&3) == 0){//左壁なし
      mouse_dir=(mouse_dir-1)&0x03;
      left_turn();
      straight();
    }
    else if(get_map(mouse_x, mouse_y, mouse_dir) == 0){	//前壁なし
      straight();
    }
    else if(get_map(mouse_x, mouse_y, (mouse_dir+1)&3) == 0){//右壁なし
      mouse_dir=(mouse_dir+1)&0x03;
      right_turn();
      straight();
    }
    else{//袋小路
      mouse_dir=(mouse_dir+2)&0x03;
      turn180();
      straight();
    }
  }
  mouse_dir=(mouse_dir+2)&0x3;
  turn180();
}

//拡張右手法
void mode1(void)
{
  make_mapdata();
  reset_map();

  //Start
  straight();
  
  while(1){
    smap[mouse_x][mouse_y]=1;
    //座標更新（柱と柱のあいだで更新）
    update_mouse_position(mouse_dir);
    //マップ更新
    if(smap[mouse_x][mouse_y]==0)
    {
      //正面
      if(get_map(mouse_x, mouse_y, mouse_dir)== 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y]>>mouse_dir)&0x01 );
      }
      else
      {	//仮想壁挿入
        set_map(mouse_x, mouse_y, mouse_dir, 1);
      }

      //右
      if(get_map(mouse_x, mouse_y, (mouse_dir+1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir+1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir+1)&3) )&0x01 );
      }
      else
      {	//仮想壁挿入
        set_map(mouse_x, mouse_y, (mouse_dir+1)&3, 1);
      }

      //左
      if(get_map(mouse_x, mouse_y, (mouse_dir-1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir-1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir-1)&3) )&0x01);
      }
      else
      {	//仮想壁挿入
        set_map(mouse_x, mouse_y, (mouse_dir-1)&3, 1);
      }

    }
    else
    {
      //正面
      if(get_map(mouse_x, mouse_y, mouse_dir)== 2)
      {
        set_map(mouse_x, mouse_y, mouse_dir, (mapdata[mouse_x][mouse_y]>>mouse_dir)&0x01 );
      }
      //右
      if(get_map(mouse_x, mouse_y, (mouse_dir+1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir+1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir+1)&3) )&0x01 );
      }
      //左
      if(get_map(mouse_x, mouse_y, (mouse_dir-1)&3)== 2)
      {
        set_map(mouse_x, mouse_y, (mouse_dir-1)&3, ( mapdata[mouse_x][mouse_y]>>((mouse_dir-1)&3) )&0x01 );
      }
    }

    printf("(%7.3f, %2d, %2d, %02X, %02X)\n", 
        (double)Index/Framerate, mouse_x, mouse_y, mapdata[mouse_x][mouse_y], map[mouse_x][mouse_y]);
    fflush(stdout);
    //スタートに戻ったら終了
    if (mouse_x==GOAL_X && mouse_y==GOAL_Y)break;

    //行動
    if(get_map(mouse_x, mouse_y, (mouse_dir+1)&3) == 0){//右壁なし
      mouse_dir=(mouse_dir+1)&0x03;
      right_turn();
      straight();
    }
    else if(get_map(mouse_x, mouse_y, mouse_dir) == 0){	//前壁なし
      straight();
    }
    else if(get_map(mouse_x, mouse_y, (mouse_dir-1)&3) == 0){//左壁なし
      mouse_dir=(mouse_dir-1)&0x03;
      left_turn();
      straight();
    }
    else{//袋小路
      mouse_dir=(mouse_dir+2)&0x03;
      turn180();
      straight();
    }
  }
  mouse_dir=(mouse_dir+2)&0x3;
  turn180();
}



int main(int argc, char** argv)
{
  int width = 180*16 + 12, height = 180*16 + 12;
  CS = cairo_image_surface_create( CAIRO_FORMAT_ARGB32, width, height );
  C = cairo_create( CS );
  
  //Start
  //draw_maze(width, height );
  //mode1();
  calc_trajectry();
  //Finsih  
  
  cairo_destroy( C );
  cairo_surface_destroy( CS );
  return 0;
}

#if 0
//2017 Clasic mouse expart final maze
char maze[33][66]={
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",//    0
    "|   |                                                           |",//15  1
    "+   +   +   +---+---+---+---+---+---+---+---+---+---+   +   +---+",//    2
    "|       |   |               |               |       |   |       |",//14  3
    "+   +---+   +   +---+---+   +   +---+---+   +   +   +---+---+   +",//    4
    "|           |   |       |       |       |       |           |   |",//13  5
    "+   +---+   +   +   +   +---+---+   +   +---+---+---+---+   +   +",//    6
    "|           |       |               |               |       |   |",//12  7
    "+   +---+---+---+---+---+---+---+---+---+---+---+   +   +---+   +",//    8
    "|   |       |       |           |       |   |       |       |   |",//11  9
    "+   +   +   +   +   +   +   +   +   +   +   +   +---+---+   +   +",//   10
    "|   |   |       |       |   |       |           |           |   |",//10 11
    "+   +   +---+---+---+---+---+---+---+---+---+---+---+   +---+   +",//   12
    "|   |       |           |               |       |           |   |",// 9 13
    "+   +---+   +   +   +---+   +   +---+   +   +   +---+   +---+   +",//   14
    "|   |       |   |       |   |       |       |               |   |",// 8 15
    "+   +   +---+   +---+   +   +   +   +---+---+---+---+   +---+   +",//   16
    "|   |   |       |       |   |       |   |                   |   |",// 7 17
    "+   +   +   +---+   +---+   +---+---+   +   +   +---+---+---+   +",//   18
    "|   |       |           |                   |               |   |",// 6 19
    "+   +---+---+   +---+---+---+---+---+---+---+---+---+---+   +   +",//   20
    "|   |           |       |   |       |   |           |       |   |",// 5 21
    "+   +   +---+---+   +   +   +   +   +   +   +   +---+   +---+   +",//   22
    "|   |       |       |           |           |   |   |       |   |",// 4 23
    "+   +---+   +   +---+---+   +---+---+   +---+   +   +---+   +   +",//   24
    "|   |       |       |   |   |       |   |       |           |   |",// 3 25
    "+   +   +---+---+   +   +---+   +   +---+   +---+   +---+---+   +",//   26
    "|   |   |           |           |           |   |   |           |",// 2 27
    "+   +   +   +---+---+   +---+---+---+---+---+   +   +   +   +   +",//   28
    "|       |   |   |   |                               |   |   |   |",// 1 29
    "+   +---+   +   +   +---+---+---+---+---+---+---+---+---+---+   +",//   30
    "|   |                                                           |",// 0 31
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+"//    32
    // 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
};
#endif
