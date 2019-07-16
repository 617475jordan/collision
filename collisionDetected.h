#pragma once
#include <iostream>
#include <iomanip>
#include <math.h>
#include <vector>
#include <algorithm>
#include <random>
using namespace  std;

struct Position
{
	double x;
	double y;
	double z;
	double angle = 0;
	double width = 0;
	double height = 0;
	double length = 0;
	double lengthBiasY = sin(angle)*length;
	double lengthBiasX = cos(angle)*length;
	double speed = 0;
};


//#define LevelCoefficient
//#define VehicleNum 10

class collisionDetected
{
public:
	collisionDetected(std::vector<Position> vehcilePosition);
	~collisionDetected();

	void   AugmentedMatrixAssignment(int rows, int cols, double a[]); //对所构造的多项式系数求解的增广矩阵赋值
	//bool AcquiringIntersectionCoordinates(unsigned char& number, double  coefficient[], double result[]);
	double Quadrature(unsigned char& number, double  coefficient[], double& initialtValue, double& finalValue);
	double GetLineLength(unsigned char& number, double  coefficient[], double& initialtValue, double& finalValue);
	bool   IsTwoCoordinateConsistent(std::vector<Position> m_currentPositon, std::string m_direction);
	bool   Derivation(unsigned char & number, double  coefficient[], double & currentPositionXvalue, double & result);
	bool   EquationSolution(unsigned char& number, double  coefficient[], unsigned char& m_iEquationSolutionNum, double result[]);
	bool   UnivariateQuadraticSolution(unsigned char& number, double  coefficient[], unsigned char& m_iEquationSolutionNum, double result[]);
	bool   OneYuanOneTimeSolution(unsigned char& number, double  coefficient[], unsigned char& m_iEquationSolutionNum, double result[]);
	//std::vector<Position> m_secondVehiclePosition;


private:

	double PowerSummationFunction(int k, int n); //x[i]次幂求和函数
	double ProductSumFunction(int k, int n); //x[i]次幂与y[i]乘积求和函数
	void   SolveEquation(int &m, double a[]); //求解方程函数
	double AdditionalFunction(double c[], int l, int& m); //供C调用

	//bool _AcquiringIntersectionCoordinates(double  coefficient[]);


	double coefficientMatrix[10][10]; //求多项式系数时需要的矩阵
	double m_fResult[3];

	std::vector<Position> m_vehiclePosition;
	//unsigned char m_levelCoefficient = 0;    // 4 a*x*x*x*x+b*x*x*x+c*x*x+d*x+e
};

