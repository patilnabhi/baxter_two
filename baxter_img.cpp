#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#define uchar unsigned char
#define _r(x, y, image) ((uchar *)((image)->imageData + ((image)->widthStep) * (x)))[3 * y + 2]
#define _g(x, y, image) ((uchar *)((image)->imageData + ((image)->widthStep) * (x)))[3 * y + 1]
#define _b(x, y, image) ((uchar *)((image)->imageData + ((image)->widthStep) * (x)))[3 * y]
#define _Ma(x, y, mat) ((mat)->data.fl + (mat)->step / 4 * (x))[y]
#define simulator 0
#define fla 1
using namespace cv;
uchar *temp;
int flag = 1;
int size_ = 0;
//float pose[2] = {0};
IplImage *head;
IplImage *left_hand;
IplImage *right_hand;
IplImage *red;
IplImage *green;
IplImage *blue;
CvMat *TRANS = cvCreateMat(3, 3, CV_32FC1);
CvMat *INVTRANS = cvCreateMat(3, 3, CV_32FC1);
CvMat *DIS = cvCreateMat(1, 4, CV_32FC1);
int WIDTH, HEIGHT, STEP;
int NUM_OBJ = 0;
#if fla == 1
uchar COLOR[3] = {60, 85, 70};
uchar COLOR_[3] = {90, 175, 255};
uchar COLOR1[3] = {0, 90, 80};
uchar COLOR2[3] = {9, 255, 245};
#else
uchar COLOR[3] = {100, 100, 0};
#endif
extern void GetPosition(IplImage *);
extern int GetColor(IplImage *);
extern void FilterColor(IplImage *, IplImage *, uchar *);
extern void FilterColor_(IplImage *, IplImage *, uchar *);
ros::Publisher pos_pub;
typedef struct pos
{
  int x, y;
  float x1;
  int x2, y2;
  uchar color;
  float angle;
}pos;
typedef struct re_pos
{
  float x, y, z;
  float x1, y1, z1;
  float angle;
  uchar color;
  int flag;
}pos_;
typedef struct rpos
{
  float x, y, z, size;
  uchar color;
  float angle;
}rpos;

extern void Get3DPos(pos *src, pos_ *dis, int num);
extern void GetRealPos(float, float, float, float, float, float, float, float, pos_*);
extern void GetRealPos_(float x, float y, float xz, float yz, float zz, float xp, float yp, float zp, pos_ *dis);
extern void thres(rpos* src, pos_ *dis);
pos *pos_head = (pos *)malloc(sizeof(pos) * 100);
pos_ *rel_pos = (pos_ *)malloc(sizeof(pos_) * 100);
rpos *f_pos = (rpos *)malloc(sizeof(rpos) * 100);
void ImageToIpl(uchar *src, IplImage *dis)
{
  for(int i = 0; i != HEIGHT; i ++)
  {
    for(int j = 0; j != WIDTH; j++)
    {
#if simulator == 1
      ((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j] = src[dis->widthStep / 3 * 3 * i + 3 * j + 2];
      ((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j + 1] = src[dis->widthStep / 3 * 3 * i + 3 * j + 1];
      ((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j + 2] = src[dis->widthStep / 3 * 3 * i + 3 * j + 0];
#else
      ((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j] = src[dis->widthStep / 3 * 4 * i + 4 * j + 0];
      ((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j + 1] = src[dis->widthStep / 3 * 4 * i + 4 * j + 1];
      ((uchar *)(dis->imageData) + (dis->widthStep) * i)[3 * j + 2] = src[dis->widthStep / 3 * 4 * i + 4 * j + 2];
#endif     
    }
  }
}

void ImageCallBack(const sensor_msgs::Image &msg)
{
  if (flag == 1)
  {
    WIDTH = msg.width;
    HEIGHT = msg.height;
    STEP = msg.step;
    size_ = msg.step * msg.height;
    temp = (uchar *)malloc(size_);
    flag ++;
    head = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
    red = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
    green = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
    blue = cvCreateImage(cvSize(msg.width, msg.height), IPL_DEPTH_8U, 3);
  }
  for(int i = 0; i != size_; i++)
  {
    temp[i] = msg.data[i];
  }
  ImageToIpl(temp, head);
  //head = cvLoadImage("frame0000.jpg");
  IplImage *pp = NULL;
  GetPosition(head);
  geometry_msgs::PoseArray points;
  geometry_msgs::Pose pose;
  for(int i = 0; i != NUM_OBJ; i++)
  {
    pose.position.x = rel_pos[i].x / 1000;
    pose.position.y = rel_pos[i].y / 1000;
    pose.position.z = rel_pos[i].angle;
    pose.orientation.x = f_pos[i].size;
    points.poses.push_back(pose);
  }
  pos_pub.publish(points);
}
int main(int arg, char** argv)
{
  _Ma(0, 0, TRANS) = 411.78954;
  _Ma(0, 1, TRANS) = 0;
  _Ma(0, 2, TRANS) = 307.70709;
  _Ma(1, 0, TRANS) = 0;
  _Ma(1, 1, TRANS) = 411.10182;
  _Ma(1, 2, TRANS) = 236.83502;
  _Ma(2, 0, TRANS) = 0;
  _Ma(2, 1, TRANS) = 0;
  _Ma(2, 2, TRANS) = 1;
  _Ma(0, 0, INVTRANS) = 0.0022042;
  _Ma(0, 1, INVTRANS) = 0;
  _Ma(0, 2, INVTRANS) = -0.74724;
  _Ma(1, 0, INVTRANS) = 0;
  _Ma(1, 1, INVTRANS) = 0.0022049;
  _Ma(1, 2, INVTRANS) = -0.57610;
  _Ma(2, 0, INVTRANS) = 0;
  _Ma(2, 1, INVTRANS) = 0;
  _Ma(2, 2, INVTRANS) = 1;
  _Ma(0, 0, DIS) = 0.01386;
  _Ma(0, 1, DIS) = -0.058;
  _Ma(0, 2, DIS) = 0.00134;
  _Ma(0, 3, DIS) = 0.00227;
  ros::init(arg, argv, "test");
  ros::NodeHandle n;
  pos_pub = n.advertise<geometry_msgs::PoseArray>("Dpos", 2);
  ros::Subscriber sub = n.subscribe("/cameras/right_hand_camera/image", 1, ImageCallBack);
  ros::spin();
  return 0;
}
void GetPosition(IplImage *src)
{
  IplImage *temp_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
  IplImage *src_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
  IplImage *gray_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *con = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  IplImage *con_;
  static IplImage *test = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
  cvZero(test);
  double min = 50;
  //IplConvKernel *elemt = NULL;
  NUM_OBJ = 0;
  //cvUndistort2(src, src_, TRANS, DIS);
  //ROS_INFO("%d", (int)_b(200, 100, src_));
  cvSmooth(src, src, CV_GAUSSIAN, 5, 5);
  FilterColor(src, blue, COLOR);
  FilterColor_(src, red, COLOR);
  cvErode(blue, temp_, NULL, 4);
  IplImage *gray = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
  cvCvtColor(temp_, gray, CV_BGR2GRAY);
  cvThreshold(gray, gray_, 20, 255, CV_THRESH_BINARY);
  //cvCanny(gray_, con, 30, 60, 3);
  con_ = cvCloneImage(gray_);
  CvMemStorage *st = cvCreateMemStorage(0);
  CvSeq *first = NULL;
  cvFindContours(con_, st, &first, sizeof(CvContour), CV_RETR_LIST);
  CvSeq *big = NULL;
  double area = 0;
  for(CvSeq *c = first; c != NULL; c = c->h_next)
  {
    if(fabs(cvContourArea(c)) < min);
      //;continue;
    if (fabs(cvContourArea(c)) >= area)
    {
      big = c;
      area = fabs(cvContourArea(c));
    }
    /*float x = 0, y = 0;
    for (int i = 0; i != c->total; i++)
    {
      CvPoint *p = CV_GET_SEQ_ELEM(CvPoint, c, i);
      x += p->x;
      y += p->y;
    }
    x /= c->total;
    y /= c->total;
    pos_head[NUM_OBJ].x = (int)x - WIDTH / 2;
    pos_head[NUM_OBJ].y = (int)(HEIGHT - y) - HEIGHT / 2;*/
    CvBox2D rect = cvMinAreaRect2(c, st);
    pos_head[NUM_OBJ].x = (int) rect.center.x - WIDTH / 2;
    pos_head[NUM_OBJ].y = -((int) rect.center.y - HEIGHT / 2);
    //pos_head[NUM_OBJ].x1 = (float) sqrt(rect.size.height * rect.size.height + rect.size.width * rect.size.width);
    pos_head[NUM_OBJ].x2 = (int) (pos_head[NUM_OBJ].x + sin(rect.angle / 180 * 3.14) * rect.size.height / 2);
    pos_head[NUM_OBJ].y2 = (int) (pos_head[NUM_OBJ].y + cos(rect.angle / 180 * 3.14) * rect.size.height / 2);
    pos_head[NUM_OBJ].angle = rect.angle;
    Get3DPos(pos_head + NUM_OBJ, rel_pos + NUM_OBJ, 1);
    f_pos[NUM_OBJ].x = rel_pos[NUM_OBJ].x;
    f_pos[NUM_OBJ].y = rel_pos[NUM_OBJ].y;
    f_pos[NUM_OBJ].z = rel_pos[NUM_OBJ].z;
    f_pos[NUM_OBJ].size = sqrt((rel_pos[NUM_OBJ].x - rel_pos[NUM_OBJ].x1) * (rel_pos[NUM_OBJ].x - rel_pos[NUM_OBJ].x1) + (rel_pos[NUM_OBJ].y - rel_pos[NUM_OBJ].y1) * (rel_pos[NUM_OBJ].y - rel_pos[NUM_OBJ].y1));
    f_pos[NUM_OBJ].color = 1;
    NUM_OBJ++;      
  }

  ROS_INFO("%d", NUM_OBJ);
  cvDrawContours(test, big, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), -1, 2, 8, cvPoint(0,0));
  ROS_INFO("%f, %f, %f", rel_pos[0].x, rel_pos[0].y, pos_head[0].angle);
  cvShowImage("gar", src);
  waitKey(1);
  cvReleaseMemStorage(&st);
  cvReleaseImage(&gray);
  cvReleaseImage(&temp_);
  cvReleaseImage(&gray_);
  cvReleaseImage(&src_);
  cvReleaseImage(&con);
  cvReleaseImage(&con_);
  
  //ROS_INFO("Detect [%d] objects", NUM_OBJ);
}
void FilterColor(IplImage *src, IplImage *dis, uchar* thre)
{
 IplImage *temp_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
 cvCvtColor(src, temp_, CV_BGR2HSV);
  for(int i = 0; i != src->height; i++)
  {
    for(int j = 0; j != src->width; j++)
    {
      if(COLOR[0] <= _b(i, j, temp_) && COLOR_[0] >= _b(i, j,temp_) && COLOR[1] <= _g(i, j, temp_) && COLOR_[1] >= _g(i, j, temp_) && COLOR[2] <= _r(i, j, temp_) && COLOR_[2] >= _r(i, j, temp_))
      {
        _r(i, j, dis) = _r(i, j, src);
        _g(i, j, dis) = _g(i, j, src);
        _b(i, j, dis) = _b(i, j, src);
      }
      else
      {
         _r(i, j, dis) = 0;
         _g(i, j, dis) = 0;
         _b(i, j, dis) = 0;
      }
    }
  }
  cvReleaseImage(&temp_);
}
void FilterColor_(IplImage *src, IplImage *dis, uchar* thre)
{
 IplImage *temp_ = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
 cvCvtColor(src, temp_, CV_BGR2HSV);
  for(int i = 0; i != src->height; i++)
  {
    for(int j = 0; j != src->width; j++)
    {
      if(COLOR1[0] <= _b(i, j, temp_) && COLOR2[0] >= _b(i, j,temp_) && COLOR1[1] <= _g(i, j, temp_) && COLOR2[1] >= _g(i, j, temp_) && COLOR1[2] <= _r(i, j, temp_) && COLOR2[2] >= _r(i, j, temp_))
      {
        _r(i, j, dis) = _r(i, j, src);
        _g(i, j, dis) = _g(i, j, src);
        _b(i, j, dis) = _b(i, j, src);
      }
      else
      {
         _r(i, j, dis) = 0;
         _g(i, j, dis) = 0;
         _b(i, j, dis) = 0;
      }
    }
  }
  cvReleaseImage(&temp_);
}
void Get3DPos(pos *src, pos_ *dis, int num)///The num is the # of Object
{
  CvMat *test = cvCreateMat(3, 3, CV_32FC1);
  _Ma(1, 1, test);
  float tempx = _Ma(0, 0, INVTRANS) * src->x + 0.015;  _Ma(0, 2, INVTRANS);
  float tempy = _Ma(1, 1, INVTRANS) * src->y + 0.058;  _Ma(1, 2, INVTRANS);
  float tempx2 = _Ma(0, 0, INVTRANS) * src->x2 + 0.015; 
  float tempy2 = _Ma(1, 1, INVTRANS) * src->y2 + 0.058; 
  GetRealPos(tempx, tempy, 0, 0, 1.0, 0, 1, 475, dis);
  GetRealPos_(tempx2, tempy2, 0, 0, 1.0, 0, 1, 475, dis);
  dis->angle = src->angle;
  dis->flag = 1;
}
void GetRealPos(float x, float y, float xz, float yz, float zz, float xp, float yp, float zp, pos_ *dis) /// xp ...zp is the porsition of a
///point on thr table, and the xz ...zz is the orientation of the normal vector of the table. 
{
  dis->y = x * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
  dis->x = -y * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
  dis->z = -(-xp * xz - yp * yz -zp * zz) / (x * xz + y * yz + zz);
}
void GetRealPos_(float x, float y, float xz, float yz, float zz, float xp, float yp, float zp, pos_ *dis) /// xp ...zp is the porsition of a
///point on thr table, and the xz ...zz is the orientation of the normal vector of the table. 
{
  dis->y1 = x * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
  dis->x1 = -y * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
  dis->z1 = -(-xp * xz - yp * yz -zp * zz) / (x * xz + y * yz + zz);
}
//void thres(rpos* src, pos_ *dis)
//{
  
//}


