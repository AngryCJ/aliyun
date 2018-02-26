---
title: 道路匹配算法--C++
thumbnail: https://img17.pixhost.org/images/212/64698033_roadmap.jpg
photos:
- https://img17.pixhost.org/images/212/64698033_roadmap.jpg
tags:
- road match
categories:
- C++
---
使用GPS信息将设备匹配到实际所处道路上。道路默认为两个点一条直线，具体的GPS信息通过[高德地图](http://lbs.amap.com/console/show/picker)取得，事先要将所需要匹配的城市道路都下载至本地。GPS的坐标是标准的地球坐标，高德和百度地图都是对实际坐标进行了偏移处理，由于我们道路采集的坐标是高德坐标系，所以在这个工程中需要将实际的GPS坐标转换成高德坐标系。

<escape><!-- more --></escape>

# 道路匹配算法原理

总体方案：采用基于道路拓扑结构的权重匹配法，主要有三方面：距离D，方向（航向）A，历史方向H。最后计算权重总和，为每条候选路段计算权重总和W，，则将具有最高权重和的候选路段作为匹配路段，将该候选路段的虚拟匹配点作为匹配点。

![流程图](https://img17.pixhost.org/images/212/64698025_roadmatch.png)

#### 1.距离匹配度

![距离匹配](https://img17.pixhost.org/images/212/64698018_touying.png)

由于GPS系统内在的误差，在一般情况下接收到的数据定位点会偏离车辆实际行驶路段。通过对定位点P作其在各个候选道路上的投影，投影点位为P0，所以GPS点到线段最短距离为点到投影距离PP0.
当投影点在道路上的投影点不在线段内，则点到线的最短距离，为点到线段两端的最短距离。

#### 2.方向匹配度

![方向匹配](https://img17.pixhost.org/images/212/64698035_direction.png)

在对候选路段进行判断时，可以通过确定误差区域在第一时间排除不想管的候选路段。由于GPS系统接受到的定位点轨迹与电子地图中的道路存在着方向上的偏差，因此需要对两者进行比较并将此作为一个重要的判断因素。
从GPS系统中接受到的数据信息可知车辆的运行方向，然后分别计算车辆在每条候选路段上的方向匹配度。假设GPS系统显示的定位点轨迹为路线L1，
L2为实际道路，则计算两条方向向量的角度差cos(α-β)作为方向匹配度的值

#### 3.历史方向匹配

在进行道路判断时，我们会将每个GPS点（也就是车辆轨迹）存储在内存空间，加入现在GPS点为P0,前四个数据点分别为P-5,P-4,P-3,P-2,P-1
我们计算P0 P-，P0 P-4   P0P-3   P0P-2,   P0P-1
五条线路与道路方向的角度差并求平均作为历史方向匹配度的值。

#### 4.权重分配

1.静态：将三个权重值平均分别为1/3,则每条道路的总得分为W=1/3*（D+A+H），或者在之后的实际上路测试不断调权重值得到对应的经验值
2.动态：
自动分配权重，先假设距离、方向、历史方向、的权重值相同。在此从第一时候开始，分别对每条候选道路的距离、方向、实时方向匹配度进行排序。然后在每时刻都将做出三个因素的最高匹配度与次高匹配度的差值。在此，设定一个阈值h，若在一个时刻某个因素差值最大，并且大于阈值h，则可以认为在该时刻，该因素的权重值占比重最大，提高其权重值。

# 源代码（C++)

#### GPS数据解析

```
//"$GPRMC,075507.00,A,3027.40466,N,11425.31127,E,0.792,,120517,,,A*7F\r\n"
/*数据详解：$GPRMC,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,<10>,<11>,<12>*hh
<1> UTC 时间，hhmmss(时分秒)格式
<2> 定位状态，A=有效定位，V=无效定位
<3>纬度ddmm.mmmm(度分)格式(前面的0也将被传输)
<4> 纬度半球N(北半球)或S(南半球)
<5>经度dddmm.mmmm(度分)格式(前面的0也将被传输)
<6> 经度半球E(东经)或W(西经)
<7>地面速率(000.0~999.9节，前面的0也将被传输)
<8>地面航向(000.0~359.9度，以正北为参考基准，前面的0也将被传输)
<9> UTC 日期，ddmmyy(日月年)格式
<10>磁偏角(000.0~180.0度，前面的0也将被传输)
<11> 磁偏角方向，E(东)或W(西)
<12>模式指示(仅NMEA01833.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效)
解析内容：
1.时间，这个是格林威治时间，是世界时间（UTC），我们需要把它转换成北京时间（BTC），BTC和UTC差了8个小时，要在这个时间基础上加8个小时。
2.
定位状态，在接收到有效数据前，这个位是‘V’，后面的数据都为空，接到有效数据后，这个位是‘A’，后面才开始有数据。
 3.
纬度，我们需要把它转换成度分秒的格式，计算方法：如接收到的纬度是：4546.40891
4546.40891/100=45.4640891可以直接读出45度, 4546.40891–45*100=46.40891,
可以直接读出46分46.40891–46 =0.40891*60=24.5346读出24秒,
所以纬度是：45度46分24秒。
4. 南北纬，这个位有两种值‘N’（北纬）和‘S’（南纬）
5. 经度的计算方法和纬度的计算方法一样
6. 东西经，这个位有两种值‘E’（东经）和‘W’（西经）
7.
速率，这个速率值是海里/时，单位是节，要把它转换成千米/时，根据：1海里=1.85公里，把得到的速率乘以1.85。
8. 航向，指的是偏离正北的角度
9. 日期，这个日期是准确的，不需要转换*/
```

#### GPS坐标转高德坐标系（火星坐标系）/百度坐标系

```C++
/*******************************GPS坐标系转换***********************************/

const double pi = 3.14159265358979324;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;
const  double x_pi = 3.14159265358979324 * 3000.0 / 180.0;

bool outOfChina(double lat, double lon)
{
    if (lon < 72.004 || lon > 137.8347)
    {
        return true;
    }

    if (lat < 0.8293 || lat > 55.8271)
    {
        return true;
    }

    return false;
}

double transformLat(double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2
* sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;

    return ret;
}

double transformLon(double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1
* sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0
/ 3.0;

    return ret;
}

/**
 * 地球坐标转换为火星坐标（WGS-84到GCJ-02）
 * World Geodetic System ==> Mars Geodetic System
 *
 * @param wgLat  地球坐标
 * @param wgLon
 *
 * mglat,mglon 火星坐标
 */
void Transform2Mars(double wgLat, double wgLon,double &mgLat,double &mgLon)
{
    if (outOfChina(wgLat, wgLon))
    {
        qDebug() << "outOfChina";
        mgLat = wgLat;
        mgLon = wgLon;
        return;
    }

    double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
    double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);
    double radLat = wgLat / 180.0 * pi;
    double magic = sin(radLat);

    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    mgLat = wgLat + dLat;
    mgLon = wgLon + dLon;
    qDebug() << "mgLat111=" << mgLat << "mgLon111=" << mgLon;
}

/**
 * 火星坐标转换为百度坐标（GCJ-02到BD-09）
 * @param gg_lat
 * @param gg_lon
 */
void BD_encrypt(double gg_lat, double gg_lon,double &bd_lat,double &bd_lon)
{
    double x = gg_lon, y = gg_lat;
    double z = sqrt(x * x + y * y) + 0.00002 * sin(y * x_pi);
    double theta = atan2(y, x) + 0.000003 * cos(x * x_pi);

    bd_lon = z * cos(theta) + 0.0065;
    bd_lat = z * sin(theta) + 0.006;
}

/**
 * 百度转火星
 * @param bd_lat
 * @param bd_lon
 */
void BD_decrypt(double bd_lat, double bd_lon,double &gg_lat,double &gg_lon)
{
    double x = bd_lon - 0.0065, y = bd_lat - 0.006;
    double z = sqrt(x * x + y * y) - 0.00002 * sin(y * x_pi);
    double theta = atan2(y, x) - 0.000003 * cos(x * x_pi);

    gg_lon = z * cos(theta);
    gg_lat = z * sin(theta);
}

```

#### 球面坐标转换成平面坐标

```C++
struct geo{
    int projectType;   //分带选择：1--->3°带投影    2--->6°带投影
    int  parameter;    //参考椭球:1---->克拉索夫斯基椭球
2---->1975国际协议椭球    3---->WGS-84椭球
    double geo_B_d;    //经度： geo_B_d--->度
    double geo_B_m;    //经度： geo_B_m--->分
    double geo_B_s;    //经度： geo_B_s--->秒
    double geo_L_d;    //维度： geo_L_d--->度
    double geo_L_m;    //纬度： geo_L_m--->分
    double geo_L_s;    //维度： geo_L_s--->秒
    double centerLong;  //中央子午线经度
    double plane_X;     //平面坐标x
    double plane_Y;     //平面坐标y
    double a;           //椭球参数a
    double b;           //椭球参数b
};

void gaussConvert(geo* m)     //坐标转换，将大地坐标转换成直角坐标
{

        double N;
        double t;
        double Eta;
        double X;
        double A0,A2,A4,A6,A8;
        double RadB;

        double Rou;
        Rou=180*3600/PI;

        double a,b,e1,e2;  //椭球参数

        LawJudge::getParameter(m);
        a=m->a;
        b=m->b;
        e1=sqrt(a*a-b*b)/a;
        e2=sqrt(a*a-b*b)/b;

        double l;
        double L0;
        LawJudge::getProjectType(m);
        L0=m->centerLong;
        double L;
        L=m->geo_L_d+m->geo_L_m/60+m->geo_L_s/3600;
        l=(L-L0)*3600;

        RadB=(m->geo_B_d+m->geo_B_m/60+m->geo_B_s/3600)*PI/180;

        N=a/sqrt(1-e1*e1*sin(RadB)*sin(RadB));
        t=tan(RadB);
        Eta=e2*cos(RadB);

        A0=1+3.0/4*e1*e1+45.0/64*pow(e1,4)+350.0/512*pow(e1,6)+11025.0/16384*pow(e1,8);
        A2=-1.0/2*(3.0/4*e1*e1+60.0/64*pow(e1,4)+525.0/512*pow(e1,6)+17640.0/16384*pow(e1,8));
        A4=1.0/4*(15.0/64*pow(e1,4)+210.0/512*pow(e1,6)+8820.0/16384*pow(e1,8));
        A6=-1.0/6*(35.0/512*pow(e1,6)+2520.0/16384*pow(e1,8));
        A8=1.0/8*(315.0/16384*pow(e1,8));
        X=a*(1-e1*e1)*(A0*RadB+A2*sin(2*RadB)+A4*sin(4*RadB)+A6*sin(6*RadB)+A8*sin(8*RadB));

        //计算平面横轴
        m->plane_X=X+N/(2*Rou*Rou)*sin(RadB)*cos(RadB)*l*l+
            N/(24*pow(Rou,4))*sin(RadB)*pow(cos(RadB),3)*(5-t*t+9*Eta*Eta+4*pow(Eta,4))*pow(l,4)+
            N/(720*pow(Rou,6))*sin(RadB)*pow(cos(RadB),5)*(61-58*t*t+pow(t,4))*pow(l,6);

        //计算平面纵轴
        m->plane_Y=N/Rou*cos(RadB)*l+
            N/(6*pow(Rou,3))*pow(cos(RadB),3)*(1-t*t+Eta*Eta)*pow(l,3)+
            N/(120*pow(Rou,5))*pow(cos(RadB),5)*(5-18*t*t+pow(t,4)+14*Eta*Eta-58*Eta*Eta*t*t)*pow(l,5);

       //计算中央经线经度
      //  centerLong=L0;

}


geo typegeo={0,0,0,0,0,0,0,0,0,0,0,0,0};
typegeo.parameter=1;
typegeo.projectType=1;
typegeo.geo_B_d=******;  //测试经度点
typegeo.geo_L_d=******;   //测试纬度点
gaussConvert(&typegeo);
double x=typegeo.plane_X;
double y=typegeo.plane_Y;
```

#### 点到线段的最短距离

```C++
double distancePointLine(double x,double y,double x1,double y1,double x2,double
y2)  //p(x,y)实际点 A(x1,y1) B(x2,y2) 点到线段的最短距离
{
    double R=(x2-x1)*(x-x1)+(y2-y1)*(y-y1);
    double D=(x2-x1)*(x2-x1)+(y2-y1)*(y2-y1);
    if(R<=0)                                            //最短距离等于AP
    {
        return sqrt(pow((x1-x),2)+pow((y1-y),2));
    }
    else if(R>=D)                                       //最短距离等于BP
    {
        return sqrt(pow((x2-x),2)+pow((y2-y),2));
    }
    else                                                //最短距离等于垂线
    {
        double r=R/D;
        double px=x1+(x2-x1)*r;
        double py=y1+(y2-y1)*r;
        return sqrt(pow((px-x),2)+pow((py-y),2));
    }

}
```

#### 两条线段的夹角

```C++
double LawJudge::straightAngle(double x,double y,double x1,double y1)
//求两条方向向量A=(x,y)和 B=(x1,y1)的夹角
{
    double A=x*x1+y*y1;
    double B;
    if(A==0)
    {
        B=1;
    }
    else
    {
        B=(sqrt(pow(x,2)+pow(y,2)))*(sqrt(pow(x1,2)+pow(y1,2))) ;
    }
    double angel=abs(A/B);
    double hudu=acos(angel);         //弧度
    double jiaodu=(180*hudu)/PI;     //角度
    return jiaodu;
}
```
