// ConsoleApplication1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
//没用c++的面向对象编程  (对象=结构体  方法=对结构体函数)
//其实用面对对象编程可能好定义一些
//VS编译器c语言居然可以中文  不该在建模搞首字母变量的   
#include <iostream>
#include <cmath>
#include <limits>
#include <opencv2/opencv.hpp>
#include "cvui.h"
using namespace cv;
using namespace std;
using namespace std::chrono;
//子弹序列改为后坐力拐点序列
const double PI = 3.14159265358979323846;
const double ScalingFactor = 1;
const double& 游戏距离到屏幕偏差放缩系数 = ScalingFactor;
const int 画布X宽度 = 900;
const int 画布Y长度 = 900;

typedef double(*概率密度函数类型)(double, double, double);
typedef double(*子弹衰减函数类型)(double);
double 正态分布随机数(double, double);
struct 玩家技术;
class ScreenPoint;
class DecisionBox;
class InflectionPoint;
class SequencePoint;
class GameLocation;
class InflectionLocation;
class SequenceLocation;

class ScreenPoint {//目标点距离准星屏幕偏差
public:
    ScreenPoint(double InX = 0, double InY = 0) :X(InX), Y(InY) {}
    double X;//目标点距离屏幕原点偏差x像素
    double Y;//目标点距离屏幕原点偏差y像素
public:
    ScreenPoint operator+(const ScreenPoint& other) {//重载加法符号
        return ScreenPoint(X + other.X, Y + other.Y);
    }
    ScreenPoint operator-(const ScreenPoint& other) {//重载减法符号
        return ScreenPoint(X - other.X, Y - other.Y);
    }
    ScreenPoint operator*(double other) {//重载乘法符号
        return ScreenPoint(X * other, Y * other);
    }
    ScreenPoint operator/(double other) {//重载除法符号
        return ScreenPoint(X / other, Y / other);
    }
    friend ostream& operator<<(ostream& output,const ScreenPoint& Point) {//重载cout输出
        output << " 点X屏幕坐标 " << Point.X << " 点Y屏幕坐标 " << Point.Y;
        return output;
    }
    ScreenPoint 向量旋转(double other) {//向量旋转other度
        return ScreenPoint(X * cos(other) - Y * sin(other), Y * cos(other) + X * sin(other));
    }
    bool operator==(const ScreenPoint& other) {
        if (X == other.X && Y == other.Y)return true;
        else return false;
    }
    Point SPtoP() {
        return Point(X, Y);
    }
};

//记录了后坐力发生剧烈转折时子弹位置与时间或者子弹在不压枪情况下子弹位置和距离第一次开火时间 
//这里直接输入所有子弹的位置与时间也可以,直接将后坐力拆成超多拐点的折现  子弹XY坐标也可以做成后坐力偏差类型  建议数组 拐点[0]={0,0,0}
class InflectionPoint {//屏幕拐点
public:
    InflectionPoint(double S_X = 0.0, double S_Y = 0.0, double T = 0.0, double V_X = 0.0, double V_Y = 0.0, double A_X = 0.0, double A_Y = 0.0)
        : Shift(S_X, S_Y), Velocity(V_X, V_Y), Accelerated(A_X, A_Y), Time(T) {}//含参构造函数
    ScreenPoint Shift;//位移
    ScreenPoint Velocity;//速度
    ScreenPoint Accelerated;//加速度
    double Time;
public:
    friend ostream& operator<<(ostream& output, const InflectionPoint& Point) {//重载cout输出
        output << " X屏幕坐标 " << Point.Shift.X << " Y屏幕坐标 " << Point.Shift.Y << " 时间 " << Point.Time;
        output << " X屏幕速度 " << Point.Velocity.X << " Y屏幕速度 " << Point.Velocity.Y;
        output << " X屏幕加速度 " << Point.Accelerated.X << " Y屏幕加速度 " << Point.Accelerated.Y << endl;
        return output;
    }
};

class GameLocation {//目标位置距离原点单位
public:
    GameLocation(double InX = 0, double InY = 0, double InZ = 10) :X(InX), Y(InY), Z(InZ) {}
    double X;//目标位置距离原点x个单位
    double Y;//目标位置距离原点y个单位
    double Z;//目标位置距离原点z个单位
public:
    GameLocation operator+(const GameLocation& other) {
        return GameLocation(X + other.X, Y + other.Y, Z + other.Z);
    }
    GameLocation operator-(const GameLocation& other) {
        return GameLocation(X - other.X, Y - other.Y, Z - other.Z);
    }
    GameLocation operator*(double other) {
        return GameLocation(X * other, Y * other, Z * other);
    }
    GameLocation operator/(double other) {
        return GameLocation(X / other, Y / other, Z / other);
    }
    ScreenPoint LocationToPoint() {//位置转变成偏差函数
        if (Z == 0) {
            cout << "Z轴为0 错误";
            return ScreenPoint(0, 0);
        }
        return ScreenPoint(X * ScalingFactor / Z, Y * ScalingFactor / Z);
    }
    friend ostream& operator<<(ostream& output, const GameLocation& Point) {
        output << " 点X游戏位置 " << Point.X << " 点Y游戏位置 " << Point.Y << " 点Z游戏位置 " << Point.Z;
        return output;
    }
    bool operator==(const GameLocation& other) {
        if (X == other.X && Y == other.Y && Z == other.Z)return true;
        else return false;
    }
};//目标位置距离原点单位

class InflectionLocation {//记录了游戏内的 实际 坐标的拐点
public:
    InflectionLocation(double S_X = 0.0, double S_Y = 0.0, double S_Z = 10.0, double T = 0.0, double V_X = 0.0, double V_Y = 0.0, double V_Z = 0.0, double A_X = 0.0, double A_Y = 0.0, double A_Z = 0.0)
        : Shift(S_X, S_Y, S_Z), Velocity(V_X, V_Y, V_Z), Accelerated(A_X, A_Y, A_Z), Time(T) {}
    GameLocation Shift;//位移
    GameLocation Velocity;//速度
    GameLocation Accelerated;//加速度
    double Time;
public:
    friend ostream& operator<<(ostream& output, const InflectionLocation& Location) {//重载cout输出
        output << " X屏幕坐标 " << Location.Shift.X << " Y屏幕坐标 " << Location.Shift.Y << " Z屏幕坐标 " << Location.Shift.Z << " 时间 " << Location.Time;
        output << " X屏幕速度 " << Location.Velocity.X << " Y屏幕速度 " << Location.Velocity.Y << " Z屏幕速度 " << Location.Velocity.Z;
        output << " X屏幕加速度 " << Location.Accelerated.X << " Y屏幕加速度 " << Location.Accelerated.Y << " Z屏幕加速度 " << Location.Accelerated.Z << endl;
        return output;
    }
}; 

class SequenceLocation {//游戏内位置拐点序列
public:
    SequenceLocation(InflectionLocation* Point, int j) {//
        for (int i = 0; i < j; i++)
            Sequence.push_back(Point[i]);
    }
    SequenceLocation(InflectionLocation Point) {
        Sequence.push_back(Point);
    }
    SequenceLocation()
    : Sequence(1, InflectionLocation()) {}
    SequenceLocation(int i)
    : Sequence(i, InflectionLocation()) {}
    vector<InflectionLocation> Sequence;
public:
    void 实际拐点位置_转速度函数() {//实际拐点位置_转速度函数  对于只有位置的拐点序列进行一阶近似  添加其速度
        for (int i = 0; i < Sequence.size(); i++) {//求数组长度
            if (Sequence[i + 1].Time == 0) {//从数组1开始搜索  因为末尾后没定义的数组默认为0  所以如果循环到末尾 结束 
                break;
            }
            Sequence[i].Velocity = (Sequence[i + 1].Shift - Sequence[i].Shift) / (Sequence[i + 1].Time - Sequence[i].Time);
            Sequence[i].Accelerated = GameLocation(0, 0, 0);
        }
    }
    void 实际拐点速度_转位置函数() {//实际拐点速度_转位置函数  对于没有位置拐点序列进行简单积分  添加其偏差
        for (int i = 0; i < Sequence.size(); i++) {//求数组长度
            if (Sequence[i + 1].Time == 0) {//从数组1开始搜索  因为末尾后没定义的数组默认为0  所以如果循环到末尾 结束 
                break;
            }
            double 时间差 = Sequence[i + 1].Time - Sequence[i].Time;//避免重复计算 下同
            Sequence[i + 1].Shift = Sequence[i].Shift + Sequence[i].Velocity * (时间差)+Sequence[i].Accelerated * (时间差) * (时间差) / 2;
        }
    }
    ScreenPoint 实际拐点型_转屏幕偏差函数(double 时间变量) {//屏幕拐点型_转屏幕偏差函数   输入时间和拐点序列  输出在T时刻的屏幕偏差
        int i;
        for (i = 0; Sequence[i + 1].Time <= 时间变量; i++)//寻找一个Time[i]<时间<Time[i+1]的位置 下同
            if (i + 2 >= Sequence.size()) {//找遍所以未找到对应  强制结束
                cout << 时间变量 << "时间变量过长 (实际拐点型_转屏幕偏差函数)" << endl;
                return (0, 0);//以后写成抛出异常
            }
        double 时间差 = 时间变量 - Sequence[i].Time;
        return ((Sequence[i].Shift + Sequence[i].Velocity * (时间差)+Sequence[i].Accelerated * (时间差) * (时间差) / 2).LocationToPoint()); //转计算游戏内位置换成屏幕偏差点
    }
    GameLocation 实际拐点型_转实际位置函数(double 时间变量) {
        int i;
        for (i = 0; Sequence[i + 1].Time <= 时间变量; i++)//寻找一个Time[i]<时间<Time[i+1]的位置 下同
            if (i + 2 >= Sequence.size()) {
                cout << 时间变量 << "时间变量过长 (实际拐点型_转实际位置函数)" << endl;
                return GameLocation(0, 0, 0);
            }
        double 时间差 = 时间变量 - Sequence[i].Time;
        return (Sequence[i].Shift + Sequence[i].Velocity * (时间差)+Sequence[i].Accelerated * (时间差) * (时间差) / 2);
    }
    ScreenPoint 实际拐点型_转屏幕速度偏差函数(double 时间变量) {
        double ZZ;//临时目标位置Z
        ScreenPoint TemporaryPointVelocity = (0, 0);
        int i;
        for (i = 0; Sequence[i + 1].Time <= 时间变量; i++)//寻找一个转折时间[i]<时间<转折时间[i+1]的位置 下同
            if (i + 2 >= Sequence.size()) {
                cout << 时间变量 << "时间变量过长 (实际拐点型_转实际位置函数)" << endl;
                return TemporaryPointVelocity;
            }
        //设X实际位移函数  (X位置+X速度*时间+X加速度*时间^2/2)
        //设Z实际位移函数  (Z位置+Z速度*时间+Z加速度*时间^2/2)
        //屏幕位置X函数为  (X实际位移)/(Z实际位移)*放缩系数
        //所以屏幕速度X函数为 上面函数对时间求导  (X实际位移导函数*Z实际位移-X实际位移*Z实际位移导函数)/(Z实际位移)^2*放缩系数
        //((X速度*Z位置-X位置*Z速度+(X加速度*Z位置-X位置*Z加速度)*时间+((X加速度*Z速度-X速度*Z加速度)/2)*时间^2 )*放缩系数)/Z实际位移^2 
        //详细解释见文档附录
        double 时间差 = 时间变量 - Sequence[i].Time;
        ZZ = Sequence[i].Shift.Z + Sequence[i].Velocity.Z * (时间差)+Sequence[i].Accelerated.Z * (时间差) * (时间差) / 2;
        TemporaryPointVelocity.X = ((Sequence[i].Velocity.X * Sequence[i].Shift.Z - Sequence[i].Velocity.Z * Sequence[i].Shift.X + (Sequence[i].Accelerated.X * Sequence[i].Shift.Z - Sequence[i].Shift.X * Sequence[i].Accelerated.Z) * 时间差 + ((Sequence[i].Accelerated.X * Sequence[i].Velocity.Z - Sequence[i].Velocity.X * Sequence[i].Accelerated.Z) / 2) * 时间差 * 时间差) * ScalingFactor) / (ZZ * ZZ);
        TemporaryPointVelocity.Y = ((Sequence[i].Velocity.Y * Sequence[i].Shift.Z - Sequence[i].Velocity.Z * Sequence[i].Shift.Y + (Sequence[i].Accelerated.Y * Sequence[i].Shift.Z - Sequence[i].Shift.Y * Sequence[i].Accelerated.Z) * 时间差 + ((Sequence[i].Accelerated.Y * Sequence[i].Velocity.Z - Sequence[i].Velocity.Y * Sequence[i].Accelerated.Z) / 2) * 时间差 * 时间差) * ScalingFactor) / (ZZ * ZZ);
        return TemporaryPointVelocity;
    }
    friend ostream& operator<<(ostream& output, const SequenceLocation& Inflection) {
        for (int i = 0; i < Inflection.Sequence.size(); i++) {//求数组长度
            output << " 第" << i << "点 " << " X位置坐标 " << Inflection.Sequence[i];
        }
        return output;
    }
};

class DecisionBox {//判定框长宽
public:
    DecisionBox(double InRX = 0, double InUY = 0, double InLX = 0, double InLY = 0) :RightUpper(InRX, InUY), LeftLower(InLX, InLY) {}
    ScreenPoint RightUpper;
    ScreenPoint LeftLower;
public:
    friend ostream& operator<<(ostream& output, const DecisionBox& Box) {
        output << " 判断框右X屏幕坐标 " << Box.RightUpper.X << " 判断框左X屏幕坐标 " << Box.LeftLower.X << " 判断框上Y屏幕坐标 " << Box.RightUpper.Y << " 判断框下Y屏幕坐标 " << Box.LeftLower.Y;
        return output;
    }
    double HitRate(概率密度函数类型 函数指针, double 方差参数) {
        return (函数指针(RightUpper.X, RightUpper.Y, 方差参数) - 函数指针(RightUpper.X, LeftLower.Y, 方差参数) - 函数指针(LeftLower.X, RightUpper.Y, 方差参数) + 函数指针(LeftLower.X, LeftLower.Y, 方差参数));
    }
};

class SizeOfBox {//目标简单判定框大小    复杂模型可以由多个框组成  这里暂时使用单个方框
public:
    SizeOfBox(double InX=10, double InY=10, double InT=0):X(InX), Y(InY), Time(InT){}
    double X;
    double Y;
    double Time;
public:
    friend ostream& operator<<(ostream& output, const SizeOfBox& Box) {//重载cout输出
        output << " 框X游戏位置大小 " << Box.X << " 框Y游戏位置大小 " << Box.Y<< " 框时间T "<< Box.Time<<endl;
        return output;
    }
};

class SequenceBox {//目标简单判定框大小序列
public:
    SequenceBox(SizeOfBox* Box, int j) {//
        for (int i = 0; i < j; i++)
            Sequence.push_back(Box[i]);
    }
    SequenceBox(SizeOfBox Box) {
        Sequence.push_back(Box);
    }
    SequenceBox()
        : Sequence(1, SizeOfBox()) {}
    SequenceBox(int i)
        : Sequence(i, SizeOfBox()) {}
    vector<SizeOfBox> Sequence;
public:
    friend ostream& operator<<(ostream& output, const SequenceBox& Box) {//重载cout输出
        for (int i = 0; i < Box.Sequence.size(); i++) {//求数组长度
            output << " 第" << i << "点 " << " X位置坐标 " << Box.Sequence[i];
        }
        return output;
    }
};

struct 玩家技术 {//定义在最开始 后面有函数用到这个结构体
    double 简单压枪倍率;
    double 简单目标跟踪倍率;
    double 压枪方向熟练程度;
    double 压枪速度熟练程度;
    double 压枪时间熟练程度;
    double 跟枪速度熟练程度;
    double 跟枪方向熟练程度;
    double 反应力;
    double 跟枪等级;
};

class SequencePoint {//屏幕拐点序列
public:
    SequencePoint(InflectionPoint* Point, int j) {//
        for (int i = 0; i < j; i++)
            Sequence.push_back(Point[i]);
    }
    SequencePoint(InflectionPoint Point) {
        Sequence.push_back(Point);
    }
    SequencePoint()
        : Sequence(1, InflectionPoint()) {}
    SequencePoint(int i)
        : Sequence(i, InflectionPoint()) {}
    vector<InflectionPoint> Sequence;
public:
    void 屏幕拐点位置_转速度函数() {//屏幕拐点位置_转速度函数  对于只有位置的拐点序列进行一阶近似  添加其速度
        for (int i = 0; i < Sequence.size(); i++) {//求数组长度
            if (Sequence[i + 1].Time == 0) {//从数组1开始搜索  因为末尾后没定义的数组默认为0  所以如果循环到末尾 结束 
                break;
            }
            Sequence[i].Velocity = (Sequence[i + 1].Shift - Sequence[i].Shift) / (Sequence[i + 1].Time - Sequence[i].Time);
            Sequence[i].Accelerated = ScreenPoint(0, 0);
        }
    }
    void 屏幕拐点速度_转位置函数() {//屏幕拐点速度_转位置函数  对于没有位置拐点序列进行简单积分  添加其偏差
        for (int i = 0; i < Sequence.size(); i++) {//求数组长度
            if (Sequence[i + 1].Time == 0) {//从数组1开始搜索  因为末尾后没定义的数组默认为0  所以如果循环到末尾 结束 
                break;
            }
            double 时间差 = Sequence[i + 1].Time - Sequence[i].Time;//避免重复计算 下同
            Sequence[i + 1].Shift = Sequence[i].Shift + Sequence[i].Velocity * (时间差)+Sequence[i].Accelerated * (时间差) * (时间差) / 2;
        }
    }
    ScreenPoint 屏幕拐点型_转屏幕偏差函数(double 时间变量) {//屏幕拐点型_转屏幕偏差函数   输入时间和拐点序列  输出在T时刻的屏幕偏差
        int i;
        for (i = 0; Sequence[i + 1].Time <= 时间变量; i++)//寻找一个Time[i]<时间<Time[i+1]的位置 下同  
            if (i + 2 >= Sequence.size()) {//找遍所以未找到对应  强制结束    i+2的解释 首先原句为while([i + 1].Time <= 时间变量)                                              i++
                cout << 时间变量 << "时间变量过长 (屏幕拐点型_转屏幕偏差函数)" << endl;//                           i++                                              
                return ScreenPoint(0, 0);//以后写成抛出异常                                           if (i + 1 >= Sequence.size())           
            }//但是 for的i++是最后执行的  所以 i++ 和 i + 1 >= Sequence.size() 和为  i + 2 >= Sequence.size()
        double 时间差 = 时间变量 - Sequence[i].Time;
        return (Sequence[i].Shift + Sequence[i].Velocity * (时间差)+Sequence[i].Accelerated * (时间差) * (时间差) / 2);
    }
    ScreenPoint 屏幕拐点型_转屏幕速度偏差函数(double 时间变量) {//输入时间和拐点序列  输出在T时刻的屏幕偏差的速度
        int i;
        for (i = 0; Sequence[i + 1].Time <= 时间变量; i++)//寻找一个Time[i]<时间<Time[i+1]的位置 下同
            if (i + 2 >= Sequence.size()) {//找遍所以未找到对应  强制结束 
                cout << 时间变量 << "时间变量过长 (屏幕拐点型_转屏幕速度偏差函数)" << endl;
                return (0, 0);//以后写成抛出异常
            }
        return (Sequence[i].Velocity + Sequence[i].Accelerated * (时间变量 - Sequence[i].Time));
    }
    friend ostream& operator<<(ostream& output, const SequencePoint& Inflection) {
        int i;
        for (i = 0; i < Inflection.Sequence.size(); i++) {//求数组长度
            output << " 第" << i << "点 " << " X屏幕坐标 " << Inflection.Sequence[i];
        }
        return output;
    }
    void 压枪函数(玩家技术 玩家技术实体, SequencePoint 后坐力序列) {//详细解释见文档
        int i;
        int j = 1;
        ScreenPoint TemporaryPointVelocity;
        double 压枪结束到拐点时间间隔;
        double 速度误差;
        double 方向误差;
        double 时间误差;
        for (i = 0; (i + j) < 后坐力序列.Sequence.size(); i++) {//遍历后坐力序列
            if (后坐力序列.Sequence[i + j].Time == 0) {//后坐力序列到达尽头  强制结束
                break;
            }
            if (后坐力序列.Sequence[i + j].Time > Sequence[i].Time) {
                压枪结束到拐点时间间隔 = 后坐力序列.Sequence[i + j].Time - Sequence[i].Time;
                TemporaryPointVelocity = (后坐力序列.Sequence[i + j].Shift - Sequence[i].Shift) / 压枪结束到拐点时间间隔;
                速度误差 = 正态分布随机数((double)(1 / 玩家技术实体.压枪速度熟练程度), 0);
                方向误差 = 正态分布随机数((double)(1 / 玩家技术实体.压枪方向熟练程度), 0) * PI;
                时间误差 = 正态分布随机数((double)(1 / 玩家技术实体.压枪时间熟练程度), 0);
                Sequence[i].Velocity = TemporaryPointVelocity.向量旋转(PI * 方向误差) * (1 + 速度误差);
                Sequence[i + 1].Shift = Sequence[i].Shift + Sequence[i].Velocity * (压枪结束到拐点时间间隔 * (1 + 时间误差));
                Sequence[i + 1].Time = Sequence[i].Time + (压枪结束到拐点时间间隔 * (1 + 时间误差));
                Sequence[i].Accelerated = (0, 0);
            }
            else {
                j++;
                i--;
            }
        }
    };
    void 跟枪函数(SequencePoint 后坐力序列, SequencePoint 压枪序列, SequenceLocation 目标移动, 玩家技术 玩家技术实体) {//详细解释见文档
        double 前速度累计量_X屏幕 = 0;
        double 前速度累计量_Y屏幕 = 0;
        ScreenPoint 临时偏差;
        ScreenPoint 临时速度偏差;
        double 速度误差;
        double 方向误差;
        double 一阶跟枪类型能力;
        double 二阶跟枪类型能力;
        int i;
        Sequence[1].Time = 玩家技术实体.反应力;
        if (玩家技术实体.跟枪等级 < 1) {
            一阶跟枪类型能力 = 玩家技术实体.跟枪等级;
            二阶跟枪类型能力 = 0;
        }
        else {
            一阶跟枪类型能力 = 1;
            二阶跟枪类型能力 = 玩家技术实体.跟枪等级;//玩家技术实体.跟枪等级最大为2
        }
        for (i = 1; i < Sequence.size(); i++) {//1阶跟枪
            速度误差 = 正态分布随机数((double)(1 / 玩家技术实体.跟枪速度熟练程度), 0);
            方向误差 = 正态分布随机数((double)(1 / 玩家技术实体.跟枪方向熟练程度), 0);

            if (后坐力序列.屏幕拐点型_转屏幕偏差函数(i * 玩家技术实体.反应力) == ScreenPoint(0, 0) && 压枪序列.屏幕拐点型_转屏幕偏差函数(i * 玩家技术实体.反应力) == ScreenPoint(0, 0) && 目标移动.实际拐点型_转屏幕偏差函数(i * 玩家技术实体.反应力) == ScreenPoint(0, 0)) {
                break;
            }
            临时偏差 = 压枪序列.屏幕拐点型_转屏幕偏差函数(i * 玩家技术实体.反应力) -后坐力序列.屏幕拐点型_转屏幕偏差函数(i * 玩家技术实体.反应力) + 目标移动.实际拐点型_转屏幕偏差函数(i * 玩家技术实体.反应力) - Sequence[i].Shift;

            临时速度偏差 = 压枪序列.屏幕拐点型_转屏幕速度偏差函数(i * 玩家技术实体.反应力) - 后坐力序列.屏幕拐点型_转屏幕速度偏差函数(i * 玩家技术实体.反应力) + 目标移动.实际拐点型_转屏幕速度偏差函数(i * 玩家技术实体.反应力);

            Sequence[i].Velocity = (临时偏差 / 玩家技术实体.反应力 + 临时速度偏差 * 一阶跟枪类型能力).向量旋转(PI * 方向误差) * (1 + 速度误差);
            Sequence[i].Accelerated = (0, 0);
            Sequence[i + 1].Time = (i + 1) * 玩家技术实体.反应力;
            Sequence[i + 1].Shift = Sequence[i].Shift + Sequence[i].Velocity * (玩家技术实体.反应力) + (Sequence[i].Accelerated / 2) * (玩家技术实体.反应力) * (玩家技术实体.反应力);
        }
    };
};

struct 目标总体 {//说实话 一些函数已经完全被序列替代了
    SequenceBox 目标游戏内大小序列= SequenceBox(10);
    SequenceLocation 移动序列= SequenceLocation(100);
    int 目标血量;
    DecisionBox 目标在屏幕位置;
};

struct 枪械 {//枪械的一个集合 包含许多基本要素
    int 武器伤害;//伤害常量
    float 武器射速;//射速
    float 耗光时间;//弹夹耗光时间
    float 放大倍率;//放大倍率
    float 换弹时间;//换弹时间
    float 持枪移动速度;//持枪时移动速度
    float 射击移动速度;//射击时移动速度
    SequencePoint 枪械子弹序列 = SequencePoint(100);//预设100发子弹的后座位置或者关键拐点位置
    子弹衰减函数类型 子弹衰减函数指针;
    double 子弹散布参数;
    概率密度函数类型 子弹概率密度函数指针;
    概率密度函数类型 子弹概率累计函数指针;
};

double 正态分布随机数(double 方差, double 期望)//见Box-Muller算法
{
    const double 双浮点最小值 = std::numeric_limits<double>::min();
    double 正态分布随机数;
    double 均匀分布随机数1, 均匀分布随机数2;
    do
    {
        均匀分布随机数1 = rand() * (1.0 / RAND_MAX);
        均匀分布随机数2 = rand() * (1.0 / RAND_MAX);
    } while (均匀分布随机数1 <= 双浮点最小值);
    正态分布随机数 = sqrt(-2.0 * log(均匀分布随机数1)) * cos(2 * PI * 均匀分布随机数2);
    return 正态分布随机数 * 方差 + 期望;
};

double 正态分布概率密度函数(double X, double Y,double 方差倒数) {//正态分布概率密度方差倒数   描述准星扩散大小   值越大越集中   (为什么不用方差  方差数小时 1/方差  误差会被放大)
    double 概率 = 0;//子弹击中X,Y的概率
    概率=(1 * 方差倒数 * 方差倒数) / (PI * 2) * exp(-((X * X + Y * Y) * 方差倒数 * 方差倒数) / 2);
    return 概率;
}

double 均匀分布概率密度(double X, double Y, double 边界) {//子弹散布累计概率密度 既子弹击中属于(-无穷,X)(-无穷,Y)大片区域的概率 现已经废弃
    double 概率 = 0;//子弹击中X,Y的概率
    if (X > (-1 * 边界) && X<边界 && Y>(-1 * 边界) && Y < 边界){
        概率 = 1 / (边界 * 边界);
    }
    return 概率;
}

double 正态分布累计概率密度(double X, double Y,double 方差倒数) {//子弹散布累计概率密度 既子弹击中属于(-无穷,X)(-无穷,Y)大片区域的概率
    double 概率 = 0;//子弹击中X,Y的概率
    概率 = (erf(X * 方差倒数 / sqrt(2)) + 1) / 2 * (erf(Y * 方差倒数 / sqrt(2)) + 1) / 2;//我是sb  这么简单的函数找了好久没找到还想自己写
    return 概率;
}

double 均匀分布累计概率密度(double X, double Y ,double 边界) {//子弹散布累计概率密度 既子弹击中属于(-无穷,X)(-无穷,Y)大片区域的概率  现已经废弃
    double 概率 = 0;//子弹击中X,Y的概率
    if (X > 边界 && Y > 边界) {
        概率 = 1;
    }
    else if (Y > 边界&& X > -1 * 边界) {
        概率 = (X + 边界) / (2 * 边界);
    }
    else if (X > 边界 && Y > -1 * 边界) {
        概率 = (Y + 边界) / (2 * 边界);
    }
    else if (X > -1 * 边界 && Y > -1 * 边界) {
        概率 = (X + 边界) * (Y + 边界) / (4 * 边界);;
    }
    else {
        概率 = 0;
    }
    return 概率;
}

double 子弹衰减(double 目标位置Z) {//子弹衰减函数   (瞎设的数值)
    double 衰减倍率;
    if (目标位置Z <= 50) {//如果与目标距离小于50伤害不变
        衰减倍率 = 1;
    }
    else if (目标位置Z <= 70) {//如果与目标距离大于50 小于70伤害乘0.7
        衰减倍率 = 0.7;
    }
    else {//如果与目标距离大于70伤害乘0.5
        衰减倍率 = 0.5;
    }
    return 衰减倍率;
}

DecisionBox 实际位置函数(SequencePoint 后坐力序列, SequencePoint 压枪序列, SequenceLocation 目标移动序列, SequencePoint 跟枪序列, SequenceBox 目标大小实体,玩家技术 玩家技术实体, double 时间变量) {//这个是重载函数  就是名字一样但参数不一样的函数叫 重载
    DecisionBox 临时变量 = DecisionBox(0, 0, 0, 0);
    GameLocation 目标大小T时刻位置 = GameLocation(10, 10, 0);
    int i = 0;
    for (i = 0; 目标大小实体.Sequence[i + 1].Time <= 时间变量; i++)//寻找一个Time[i]<时间<Time[i+1]的位置 下同
        if (i + 2 >= 目标大小实体.Sequence.size()) {//找遍所以未找到对应  强制结束 
            cout << 时间变量 <<"时间变量过长 (实际位置函数)" << endl;
            break;
        }
    ScreenPoint 后坐力偏差 = 后坐力序列.屏幕拐点型_转屏幕偏差函数(时间变量);
    ScreenPoint 压枪偏差 = 压枪序列.屏幕拐点型_转屏幕偏差函数(时间变量);
    GameLocation 目标移动位置 = 目标移动序列.实际拐点型_转实际位置函数(时间变量);
    ScreenPoint 跟枪偏差 = 跟枪序列.屏幕拐点型_转屏幕偏差函数(时间变量);
    if (目标移动位置.Z == 0) {//目标与玩家z轴位置为0 即完全贴合 这是不可能的 报错并返回
        return 临时变量;
    }
    目标大小T时刻位置.X = 目标大小实体.Sequence[i].X;
    目标大小T时刻位置.Y = 目标大小实体.Sequence[i].Y;
    临时变量.LeftLower = (目标移动位置 - 目标大小T时刻位置 / 2).LocationToPoint() - 后坐力偏差 + 压枪偏差 - 跟枪偏差;
    临时变量.RightUpper = (目标移动位置 + 目标大小T时刻位置 / 2).LocationToPoint() - 后坐力偏差 + 压枪偏差 - 跟枪偏差;
    return 临时变量;
};

double 复杂模型累计概率形(枪械 枪械实体, 目标总体 目标实体,玩家技术 玩家实体, int 运算精度) {//无压枪 目标无位移 
    double 临时伤害总和变量 = 0;
    double 时间变量 = 0;
    double 命中概率 = 0;
    int i;
    SequencePoint 后座序列 = 枪械实体.枪械子弹序列;
    SequencePoint 压枪序列 = SequencePoint(100);
    SequenceLocation 移动序列 = 目标实体.移动序列;
    ScreenPoint 移动屏幕序列 = (0, 0);
    SequencePoint 跟枪序列 = SequencePoint(500);
    DecisionBox 目标边框位置 = DecisionBox(0, 0, 0, 0);
    后座序列.屏幕拐点位置_转速度函数();
    压枪序列.压枪函数(玩家实体, 后座序列);
    移动序列.实际拐点位置_转速度函数();
    跟枪序列.跟枪函数(后座序列, 压枪序列, 移动序列, 玩家实体);
    for (i = 0; i < 枪械实体.耗光时间 * 运算精度; i++) {
        时间变量 = (double)i / (double)运算精度;
        目标边框位置 = 实际位置函数(后座序列, 压枪序列, 移动序列, 跟枪序列, 目标实体.目标游戏内大小序列, 玩家实体, 时间变量);
        命中概率 = 目标边框位置.HitRate(枪械实体.子弹概率累计函数指针, 枪械实体.子弹散布参数);
        临时伤害总和变量 = 临时伤害总和变量 + 命中概率 * 枪械实体.武器伤害 * 枪械实体.武器射速 * 枪械实体.子弹衰减函数指针(移动序列.实际拐点型_转实际位置函数(时间变量).Z) / (double)运算精度;
        cout<<" 总伤害 " << 临时伤害总和变量<< 目标边框位置<< "时间" << 时间变量<<endl;
    }
    return 临时伤害总和变量;
};

void 目标运动绘制函数(SequenceLocation 目标移动, double Time, Mat& frame, ScreenPoint Now, ScreenPoint 中心, ScreenPoint 目标移动偏差);
void 显示函数(SequencePoint 后坐力拐点, SequencePoint 压枪拐点, SequenceLocation 目标移动, SequencePoint 跟枪拐点)
{
    // 创建窗口
    int i;
    int j;
    for (i = 0; 后坐力拐点.Sequence[i + 1].Time != 0; i++) {
    }
    cout << i;
    namedWindow("Animation", WINDOW_NORMAL);

    // 设置动画帧率
    int fps = 144;
    int delay = 1000 / fps;

    // 计算动画总时长
    double totalTime = 后坐力拐点.Sequence[i].Time - 后坐力拐点.Sequence[0].Time;

    // 记录动画开始时间
    auto start = high_resolution_clock::now();
    ScreenPoint 准心距中心偏移 = ScreenPoint(0, 0);
    ScreenPoint 中心画面坐标 = ScreenPoint((double)画布X宽度 / 2, (double)画布Y长度 / 2);

    // 循环播放动画
    while (true)
    {
        // 计算当前时间点
        auto now = high_resolution_clock::now();
        double elapsed = duration_cast<duration<double>>(now - start).count();
        double Time = fmod(elapsed, totalTime);// a/b的余数
        // 清空画布
        Mat frame(画布X宽度, 画布Y长度, CV_8UC3, Scalar(255, 255, 255));
        //绘制准星
        line(frame, Point(画布X宽度 / 2, 画布Y长度 / 2 - 20), Point(画布X宽度 / 2, 画布Y长度 / 2 + 20), Scalar(0, 0, 0), 2, LINE_AA);
        line(frame, Point(画布X宽度 / 2 - 20, 画布Y长度 / 2), Point(画布X宽度 / 2 + 20, 画布Y长度 / 2), Scalar(0, 0, 0), 2, LINE_AA);

        // 计算当前点位置
        ScreenPoint 后坐力偏差 = 后坐力拐点.屏幕拐点型_转屏幕偏差函数(Time);
        ScreenPoint 压枪偏差 = 压枪拐点.屏幕拐点型_转屏幕偏差函数(Time);
        ScreenPoint 目标移动偏差 = 目标移动.实际拐点型_转实际位置函数(Time).LocationToPoint();
        ScreenPoint 跟枪偏差 = 跟枪拐点.屏幕拐点型_转屏幕偏差函数(Time);

        //计算现在点距离开始点坐标
        准心距中心偏移 = 压枪偏差 - 后坐力偏差 - 跟枪偏差;
        // 绘制后坐力直线
        for (j = 0; 后坐力拐点.Sequence[j + 1].Time < Time; j++) {
            line(frame, ((后坐力拐点.Sequence[j].Shift + 准心距中心偏移) * (-1) + 中心画面坐标).SPtoP(), ((后坐力拐点.Sequence[j + 1].Shift + 准心距中心偏移) * (-1) + 中心画面坐标).SPtoP(), Scalar(0, 0, 0), 2, LINE_AA);
        }
        line(frame, ((后坐力拐点.Sequence[j].Shift + 准心距中心偏移) * (-1) + 中心画面坐标).SPtoP(), ((后坐力偏差 + 准心距中心偏移) * (-1) + 中心画面坐标).SPtoP(), Scalar(0, 0, 0), 2, LINE_AA);
        //绘制压枪曲线
        for (j = 0; 压枪拐点.Sequence[j + 1].Time < Time; j++) {
            line(frame, ((压枪拐点.Sequence[j].Shift - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), ((压枪拐点.Sequence[j + 1].Shift - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), Scalar(0, 0, 255), 2, LINE_AA);
        }
        line(frame, ((压枪拐点.Sequence[j].Shift - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), ((压枪偏差 - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), Scalar(0, 0, 255), 2, LINE_AA);
        //绘制移动近似曲线
        目标运动绘制函数(目标移动, Time, frame, 准心距中心偏移, 中心画面坐标, 目标移动偏差);
        //绘制跟枪曲线
        for (j = 0; 跟枪拐点.Sequence[j + 1].Time < Time; j++) {
            line(frame, ((跟枪拐点.Sequence[j].Shift - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), ((跟枪拐点.Sequence[j + 1].Shift - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), Scalar(255, 0, 0), 2, LINE_AA);
        }
        line(frame, ((跟枪拐点.Sequence[j].Shift - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), ((跟枪偏差 - 跟枪偏差) * (-1) + 中心画面坐标).SPtoP(), Scalar(255, 0, 0), 2, LINE_AA);

        // 显示画面
        imshow("Animation", frame);

        // 等待一段时间
        if (waitKey(delay) == 27) break;
    }
    return;
};

void 目标运动绘制函数(SequenceLocation 目标移动, double Time, Mat& frame, ScreenPoint Now, ScreenPoint 中心, ScreenPoint 目标移动偏差) {
    int j;
    for (j = 0; 目标移动.Sequence[j + 1].Time < Time; j++) {
        
        line(frame, ((目标移动.Sequence[j].Shift.LocationToPoint() + Now) * (-1) + 中心).SPtoP(), ((目标移动.Sequence[j + 1].Shift.LocationToPoint() + Now) * (-1) + 中心).SPtoP(), Scalar(0, 255, 0), 2, LINE_AA);
    }
    line(frame, ((目标移动.Sequence[j].Shift.LocationToPoint() + Now) * (-1) + 中心).SPtoP(),((目标移动偏差 + Now) * (-1) + 中心).SPtoP(), Scalar(0, 255, 0), 2, LINE_AA);
    return;
}

int main()
{
    srand(time(nullptr));
    枪械 枪;
    目标总体 目标;
    玩家技术 你;
    int i = 0;
    char 结束字符;
    {
        枪.武器伤害 = 18;
        枪.武器射速 = 7;
        枪.耗光时间 = 3.7;
        枪.子弹散布参数 = 2;//数字越大越准
        枪.子弹概率累计函数指针 = 正态分布累计概率密度;
        枪.子弹衰减函数指针 = 子弹衰减;
        枪.枪械子弹序列.Sequence[0] = { 0,0,0 };
        枪.枪械子弹序列.Sequence[1] = { -22,87,0.566 };
        枪.枪械子弹序列.Sequence[2] = { 8,140,0.866 };
        枪.枪械子弹序列.Sequence[3] = { -58,257,2.1 };
        枪.枪械子弹序列.Sequence[4] = { -19,305,3.06 };
        枪.枪械子弹序列.Sequence[5] = { -28,314,3.16 };
        枪.枪械子弹序列.Sequence[6] = { -6,332,3.8 };
    }
    {
        目标.目标血量 = 150;
        目标.移动序列.Sequence[0] = { 0,0,10,0 };
        目标.移动序列.Sequence[1] = { -100,100,10,0.5 };
        目标.移动序列.Sequence[2] = { -200,200,10,1.0 };
        目标.移动序列.Sequence[3] = { -300,300,10,1.5 };
        目标.移动序列.Sequence[4] = { -400,400,10,2.0 };
        目标.移动序列.Sequence[5] = { -500,500,10,2.5 };
        目标.移动序列.Sequence[6] = { -600,600,10,3.0 };
        目标.移动序列.Sequence[7] = { -700,700,10,3.5 };
        目标.移动序列.Sequence[8] = { -800,800,10,4.0 };
        目标.目标游戏内大小序列.Sequence[0] = { 100.0,100.0,0 };
        目标.目标游戏内大小序列.Sequence[1] = { 100.0,100.0,10.0 };
    }
    {
        你.压枪方向熟练程度 = 10;
        你.压枪时间熟练程度 = 10;
        你.压枪速度熟练程度 = 10;
        你.反应力 = 0.1;
        你.简单压枪倍率 = 1;
        你.简单目标跟踪倍率 = 1;
        你.跟枪方向熟练程度 = 10;
        你.跟枪速度熟练程度 = 10;
        你.跟枪等级 = 1;
    }  
    复杂模型累计概率形(枪, 目标, 你, 1000);
    SequencePoint 后座序列 = 枪.枪械子弹序列;
    SequencePoint 压枪序列 = SequencePoint(100);
    SequenceLocation 移动序列 = 目标.移动序列;
    ScreenPoint 移动屏幕序列 = (0, 0);
    SequencePoint 跟枪序列 = SequencePoint(500);
    DecisionBox 目标边框位置 = DecisionBox(0, 0, 0, 0);
    后座序列.屏幕拐点位置_转速度函数();
    压枪序列.压枪函数(你, 后座序列);
    移动序列.实际拐点位置_转速度函数();
    跟枪序列.跟枪函数(后座序列, 压枪序列, 移动序列, 你);
    显示函数(后座序列, 压枪序列, 移动序列, 跟枪序列);
}
