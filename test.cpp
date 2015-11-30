#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
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
#else
uchar COLOR[3] = {100, 100, 0};
#endif
extern void GetPosition(IplImage *);
extern int GetColor(IplImage *);
extern void FilterColor(IplImage *, IplImage *, uchar *);
typedef struct pos
{
  int x, y;
  float angle;
}pos;
typedef struct re_pos
{
  float x, y, z;
  float angle;
}pos_;

extern void Get3DPos(pos *src, pos_ *dis, int num);
extern void GetRealPos(float, float, float, float, float, float, float, float, pos_*);
pos *pos_head = (pos *)malloc(sizeof(pos) * 100);
pos_ *rel_pos = (pos_ *)malloc(sizeof(pos_) * 100);
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
  for(int i = 0; i != NUM_OBJ; i++)
  {
    //ROS_INFO("Obj %d 's position is : x = %d, y = %d", i + 1, pos_head[i].x, pos_head[i].y);
  }
  //ROS_INFO("I heard: [%d]",  );
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
  _Ma(0, 0, INVTRANS) = 0.0024842;
  _Ma(0, 1, INVTRANS) = 0;
  _Ma(0, 2, INVTRANS) = -0.74724;
  _Ma(1, 0, INVTRANS) = 0;
  _Ma(1, 1, INVTRANS) = 0.0024249;
  _Ma(1, 2, INVTRANS) = -0.57610;
  _Ma(2, 0, INVTRANS) = 0;
  _Ma(2, 1, INVTRANS) = 0;
  _Ma(2, 2, INVTRANS) = 1;
  _Ma(0, 0, DIS) = -0.00873;
  _Ma(0, 1, DIS) = -0.01703;
  _Ma(0, 2, DIS) = 0.00336;
  _Ma(0, 3, DIS) = 0.00419;
  ros::init(arg, argv, "test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/cameras/right_hand_camera/image", 10, ImageCallBack);
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
  cvUndistort2(src, src_, TRANS, DIS);
  //ROS_INFO("%d", (int)_b(200, 100, src_));
  cvSmooth(src_, src_, CV_GAUSSIAN, 5, 5);
  FilterColor(src_, blue, COLOR);
  cvErode(blue, temp_, NULL, 2);
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
    pos_head[NUM_OBJ].y = (int) rect.center.y - HEIGHT / 2;
    pos_head[NUM_OBJ].angle = rect.angle;
    Get3DPos(pos_head + NUM_OBJ, rel_pos + NUM_OBJ, 1);
    NUM_OBJ++;
        
  }
  ROS_INFO("%d", NUM_OBJ);
  cvDrawContours(test, big, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), -1, 2, 8, cvPoint(0,0));
  ROS_INFO("%d, %d, %f", pos_head[0].x, pos_head[0].y, pos_head[0].angle);
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
void Get3DPos(pos *src, pos_ *dis, int num)///The num is the # of Object
{
  CvMat *test = cvCreateMat(3, 3, CV_32FC1);
  _Ma(1, 1, test);
  float tempx = _Ma(0, 0, INVTRANS) * src->x + _Ma(0, 2, INVTRANS);
  float tempy = _Ma(1, 1, INVTRANS) * src->y + _Ma(1, 2, INVTRANS);
  GetRealPos(tempx, tempy, 0, 0, 1.0, 0, 1, 775.00, dis);
  dis->angle = src->angle;
}
void GetRealPos(float x, float y, float xz, float yz, float zz, float xp, float yp, float zp, pos_ *dis) /// xp ...zp is the porsition of a
///point on thr table, and the xz ...zz is the orientation of the normal vector of the table. 
{
  dis->x = x * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
  dis->y = y * (xp * xz + yp * yz + zp * zz) / (x * xz + y * yz + zz);
  dis->z = -(-xp * xz - yp * yz -zp * zz) / (x * xz + y * yz + zz);
}


