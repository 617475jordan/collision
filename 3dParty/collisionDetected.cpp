#include "collisionDetected.h"
#include <omp.h>
#include <time.h>
#include <math.h>

collisionDetected::collisionDetected(std::vector<collisionDetectedPosition> vehcilePosition)
{
	//m_levelCoefficient = LevelCoefficient;
	m_vehiclePosition = vehcilePosition;
	//double coefficientMatrix[10][10] = { 0 }; //求多项式系数时需要的矩阵
	//double m_fResult[3] = { 0 };
	//m_fResult[3] = { 0 };
}


collisionDetected::~collisionDetected()
{
	//coefficientMatrix[10][10] = { 0 }; //求多项式系数时需要的矩阵
	//m_fResult[2] = { 0 };
	//m_levelCoefficient = 0;
	//double coefficientMatrix[10][10] = { 0 }; //求多项式系数时需要的矩阵
	//double m_fResult[3] = { 0 };
	m_vehiclePosition.clear();
}

void collisionDetected::AugmentedMatrixAssignment(int rows, int cols, double a[])
{
	//#pragma  omp parallel for
	for (int i = 1; i <= cols; i++)
	{
		//#pragma  omp parallel sections
		//		{
		//#pragma omp section
		//			{
		for (int j = 1; j <= cols; j++)
		{
			coefficientMatrix[i][j] = PowerSummationFunction(i + j - 2, rows);
		}
		//			}
		//#pragma  omp section
		//			{
		coefficientMatrix[i][cols + 1] = ProductSumFunction(i - 1, rows);
		/*}*/
	/*}*/
	}
	coefficientMatrix[1][1] = (int)rows;
	SolveEquation(cols, a);
}

double collisionDetected::PowerSummationFunction(int cols, int rows)
{
	double m_fSum = 0;
	//#pragma  omp parallel for reduction(+:m_fSum)
	for (int i = 0; i < rows; i++)
	{
		m_fSum += InvPow(m_vehiclePosition[i].x, cols);
	}
	return m_fSum;
}

double collisionDetected::ProductSumFunction(int k, int n)
{
	double m_fSum = 0;
	//#pragma  omp parallel for  reduction(+:m_fSum)
	for (int i = 0; i < n; i++)
	{
		m_fSum += m_vehiclePosition[i].y * InvPow(m_vehiclePosition[i].x, k);
	}
	return m_fSum;
}

void collisionDetected::SolveEquation(int &m, double a[])
{
	//#pragma  omp parallel for
	for (int k = 1; k < m; k++) //消元过程
	{
		//#pragma  omp parallel for
		for (int i = k + 1; i < m + 1; i++)
		{

			//#pragma omp critical
				//		{
			double p1=0;
			if (coefficientMatrix[k][k] != 0)
			{
				p1 = coefficientMatrix[i][k] / coefficientMatrix[k][k];
			}
			//#pragma  omp parallel for
			for (int j = k; j < m + 2; j++)
			{
				coefficientMatrix[i][j] = coefficientMatrix[i][j] - coefficientMatrix[k][j] * p1;
			}
			//}

		}
	}

	a[m] = coefficientMatrix[m][m + 1] / coefficientMatrix[m][m];
	//#pragma  omp parallel for 
	for (int l = m - 1; l >= 1; l--)
	{
		//回代求解
		a[l] = (coefficientMatrix[l][m + 1] - AdditionalFunction(a, l + 1, m)) / coefficientMatrix[l][l];
	}
}

double collisionDetected::AdditionalFunction(double c[], int l, int& m)
{
	//clock_t m_time = clock();
	double m_fSum = 0;
	//#pragma  omp parallel for
	for (int i = l; i <= m; i++)
	{
		m_fSum += coefficientMatrix[l - 1][i] * c[i];
	}
	//std::cout << clock() - m_time << "ms" << std::endl;
	return m_fSum;
}



float collisionDetected::InvSqrt(float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);
	x = *(float*)&i;
	x = x * (1.5f - xhalf * x*x);
	return x;
}

double collisionDetected::InvPow(double n, int m)
{
	if (n == 0)
	{
		return 0;
	}
	else if (m == 1)
	{
		return n;
	}
	else if (m == 0)
	{
		return 1;
	}
	else if (m == -1)
	{
		return 1.0 / n;
	}

	if (m % 2 == 0)
	{
		double m_dTemp = InvPow(n, m / 2);
		return m_dTemp * m_dTemp;
	}
	else
	{
		double m_dTemp = InvPow(n, (m - 1) / 2);
		return m_dTemp * m_dTemp*n;
	}
	return 0;

}

//bool collisionDetected::AcquiringIntersectionCoordinates(unsigned char& number, double coefficient[], double result[])
//{
//	/*coefficient[4] = 1.0/3.0;
//	coefficient[3] = -2/2.0;
//	coefficient[2] = 1;*/
//	if (number == 1)
//	{
//		result[0] = 0;
//		result[1] = 0;
//		return false;
//	}
//	else if (number == 2)
//	{
//		result[0] = coefficient[1];
//		result[1] = coefficient[1];
//		return false;
//	}
//	else if (number == 3)
//	{
//		bool m_bFlag = _AcquiringIntersectionCoordinates(coefficient);
//		result[0] = m_fResult[0];
//		result[1] = m_fResult[1];
//		return m_bFlag;
//	}
//	else if (number == 4)
//	{
//		if (coefficient[4] == 0)
//		{
//			bool m_bFlag = _AcquiringIntersectionCoordinates(coefficient);
//			result[0] = m_fResult[0];
//			result[1] = m_fResult[1];
//			return m_bFlag;
//		}
//		else
//		{
//			//std::cout << std::endl;
//			//std::cout << std::endl;
//			/*for (unsigned int i = 1; i <= 4; i++)
//			{
//			std::cout <<"("<< coefficient[i] << ")*pow(x0,"<<i-1<<")+";
//			}*/
//			std::cout << std::endl;
//			double b = 2 * coefficient[3];
//			double a = 3 * coefficient[4];
//			double c = coefficient[2];
//			double delat = b * b - 4 * a*c;
//			if (delat < 0)
//			{
//				return false;
//			}
//			else
//			{
//				m_fResult[0] = (-1 * b + sqrt(delat)) / (2 * a);
//				m_fResult[1] = (-1 * b - sqrt(delat)) / (2 * a);
//				result[0] = m_fResult[0];
//				result[1] = m_fResult[1];
//				return true;
//			}
//		}
//	}
//	else
//	{
//		return false;
//	}
//}

double collisionDetected::Quadrature(unsigned char& number, double coefficient[], double& initialtValue, double& finalValue)
{
	double m_quadratureSum = 0;
	if (number >= 1)
	{
		//#pragma  omp parallel for reduction(+:m_quadratureSum)
		for (int i = 0; i < number; i++)
		{
			m_quadratureSum += 1.0 / (i + 1)*coefficient[i + 1] * (InvPow(finalValue, i + 1) - InvPow(initialtValue, i + 1));
		}
	}

	return m_quadratureSum;
}



double collisionDetected::GetLineLength(unsigned char& number, double coefficient[], double& initialtValue, double& finalValue)
{
	if (number <= 2)
	{
		return 0;
	}
	double m_fStepPrecision[6] = { 0.001, 0.01, 0.1, 1, 10, 100 };
	//double m_time = clock();
	double m_fGetLineLength = 0;
	double m_fTwoValueGap = abs(initialtValue - finalValue);

	unsigned int m_uCount = 0;
	while (m_fTwoValueGap >= 10)
	{
		if ((int)m_fTwoValueGap % 10 >= 10)
		{
			m_uCount++;
		}
		m_fTwoValueGap /= 10;
	}
	if (m_uCount >= 5)
	{
		m_uCount = 5;
	}
	//clock_t m_time = clock();
	if (initialtValue < finalValue)
	{
		double m_fMinPosition = (initialtValue < finalValue) ? initialtValue : finalValue;
		double m_fMaxPosition = (initialtValue > finalValue) ? initialtValue : finalValue;

		for (double i = m_fMinPosition; i <= m_fMaxPosition; i = i + m_fStepPrecision[m_uCount])
		{

			//#pragma  omp parallel for  reduction(+:m_fGetLineLength)
			for (int j = 1; j < number; j++)
			{
				double m_startPositionYValue = 0;
				double m_finalPositionYValue = 0;
				//#pragma omp parallel sections
					//			{
				//#pragma omp section
						//			{
				m_startPositionYValue += coefficient[j] * InvPow(i, j);
				//			}
		//#pragma omp section
				//			{
				m_finalPositionYValue += coefficient[j] * InvPow(i + m_fStepPrecision[m_uCount], j);
				//			}
					//	}
						//m_fGetLineLength += sqrt(abs(pow(m_fStepPrecision[m_uCount], 2) + pow(m_startPositionYValue - m_finalPositionYValue, 2)));
				m_fGetLineLength += InvSqrt(float(abs(InvPow(m_fStepPrecision[m_uCount], 2)) + InvPow(m_startPositionYValue - m_finalPositionYValue, 2)));
			}

		}
		//std::cout << clock() - m_time << "ms" << std::endl;
	}
	/*for (unsigned int i = 0; i < number&&number >= 0; i++)
	{
	m_quadratureSum += coefficient[i + 1] * (pow(finalValue, i ) - pow(initialtValue, i ));
	}
	m_quadratureSum = abs(m_quadratureSum) + abs(initialtValue - finalValue);*/
	/*double PI = 3.1415926535898;
	m_quadratureSum =  cos(PI/4);*/
	/*m_quadratureSum = 0.5* log(abs(1.0 / cos(finalValue) + tan(finalValue))) / log(10.0);
	m_quadratureSum = (0.5/cos(finalValue)*tan(finalValue) + 0.5* log (abs(1.0/cos(finalValue) + tan(finalValue))) /log(10.0))-
	(0.5/cos(initialtValue)*tan(initialtValue) + 0.5*log(abs(1.0/cos(initialtValue) + tan(initialtValue)))/log(10.0));*/
	//std::cout << 1.0*(clock() - m_time )<< "ms" << std::endl;

	return m_fGetLineLength;
}

//bool collisionDetected::_AcquiringIntersectionCoordinates(double coefficient[])
//{
//	if (coefficient[3] == 0)
//	{
//		return false;
//	}
//	m_fResult[0] = -1.0*coefficient[2] / (2.0*coefficient[3]);
//	m_fResult[1] = m_fResult[0];
//	return true;
//}

bool collisionDetected::IsTwoCoordinateConsistent(std::vector<collisionDetectedPosition> m_currentPositon, std::string m_direction)
{
	bool m_bFlag = false;
	for (unsigned int i = 0; i < m_currentPositon.size() - 1 && m_currentPositon.size()>1; i++)
	{
		double m_currentPositionValue = 0;
		double m_followPositionValue = 0;
		if (m_direction == "col")
		{
			m_currentPositionValue = m_currentPositon[i].x;
			m_followPositionValue = m_currentPositon[i + 1].x;
		}
		else if (m_direction == "row")
		{
			m_currentPositionValue = m_currentPositon[i].y;
			m_followPositionValue = m_currentPositon[i + 1].y;
		}
		else
		{
			return false;
		}


		if (m_currentPositionValue != m_followPositionValue)
		{
			return true;

		}
		else
		{
			m_bFlag = false;
		}
	}
	return m_bFlag;
}

bool collisionDetected::Derivation(unsigned char & number, double coefficient[], double & currentPositionXvalue, double & result)
{

	if (number <= 1)
	{
		return false;
	}
	else
	{
		double m_derivationSum = 0;

		//#pragma  omp parallel for
		for (int i = 2; i <= number; i++)
		{
			m_derivationSum = m_derivationSum + coefficient[i] * (i - 1)*InvPow(currentPositionXvalue, i - 1);

		}
		result = m_derivationSum;
		return true;
	}
}

bool collisionDetected::EquationSolution(unsigned char & number, double coefficient[], unsigned char& m_iEquationSolutionNum, double result[])
{
	if (number <= 1 || number > 4)
	{
		return false;
	}
	else if (number == 2)
	{
		bool m_bFlag = false;
		m_bFlag = OneYuanOneTimeSolution(number, coefficient, m_iEquationSolutionNum, result);
		return m_bFlag;
	}
	else if (number == 3)
	{
		bool m_bFlag = false;
		m_bFlag = UnivariateQuadraticSolution(number, coefficient, m_iEquationSolutionNum, result);
		return m_bFlag;
	}
	else if (number == 4)
	{
		if (coefficient[4] == 0)
		{
			bool m_bFlag = false;
			m_bFlag = UnivariateQuadraticSolution(number, coefficient, m_iEquationSolutionNum, result);
			return m_bFlag;
		}
		else
		{
			double a = coefficient[4];
			double b = coefficient[3];
			double c = coefficient[2];
			double d = coefficient[1];
			double A = InvPow(b, 2) - 3 * a * c;
			double B = b * c - 9 * a * d;
			double C = InvPow(c, 2) - 3 * b * d;
			double delta = InvPow(B, 2) - 4 * A*C;
			if (delta < 0 && A>0)
			{
				double T = (2 * A*b - 3 * a * B) / (2 * InvSqrt(float(InvPow(A, 3))));
				double theta = acos(T);
				//#pragma omp parallel sections
				//				{
				//#pragma  omp section
				//					{
				result[0] = (-b - 2 * InvSqrt(float(A))*cos(theta / 3.0)) / (3 * a);
				//					}
				//#pragma  omp section
				//					{
				result[1] = (-b + InvSqrt(float(A))*(cos(theta / 3.0) + InvSqrt(3.0)*sin(theta / 3.0))) / (3 * a);
				//					}
				//#pragma  omp section
				//					{
				result[2] = (-b + InvSqrt(float(A))*(cos(theta / 3.0) - InvSqrt(3.0)*sin(theta / 3.0))) / (3 * a);
				//	}
				//}
				m_iEquationSolutionNum = 3;

				return true;
			}
			else if (delta == 0)
			{
				double k = B / A;
				//#pragma omp parallel sections
				//				{
				//#pragma  omp section
				//					{
				result[0] = (-b / a) + k;
				//					}
				//#pragma  omp section
				//					{
				result[1] = (-k);
				//					}
				//#pragma  omp section
				//					{
				result[2] = (-k);
				//	}
				//}
				m_iEquationSolutionNum = 2;
				return true;
			}
			else if (delta > 0)
			{
				double y1 = A * b + 3 * a * (-B + InvSqrt(float(delta)))*0.5;
				double y2 = A * b + 3 * a * (-B - InvSqrt(float(delta)))*0.5;
				result[0] = (-B - (pow(y1, 1.0 / 3.0) + pow(y2, 1.0 / 3.0))) / (3 * a);
				m_iEquationSolutionNum = 1;
				return true;
			}
			else if (A == B && A == 0 && B == 0)
			{
				result[0] = (-c) / a;
				m_iEquationSolutionNum = 1;
				return true;
			}
		}
	}
	return false;
}

bool collisionDetected::UnivariateQuadraticSolution(unsigned char& number, double coefficient[], unsigned char& m_iEquationSolutionNum, double result[])
{
	if (number >= 3 && coefficient[3] != 0)
	{
		double a = coefficient[3];
		double b = coefficient[2];
		double c = coefficient[1];
		double delta = InvPow(b, 2) - 4 * a*c;
		if (delta < 0)
		{
			return false;
		}
		else if (delta == 0)
		{
			result[0] = result[1] = (-b) / (2 * a);
			m_iEquationSolutionNum = 1;
			return true;
		}
		else if (delta > 0)
		{
			result[0] = (-b + InvSqrt(float(delta))) / (2 * a);
			result[1] = (-b - InvSqrt(float(delta))) / (2 * a);
			m_iEquationSolutionNum = 2;
			return true;
		}
	}
	else
	{
		bool m_bFlag = false;
		m_bFlag = OneYuanOneTimeSolution(number, coefficient, m_iEquationSolutionNum, result);
		return m_bFlag;
	}
	return false;
}

bool collisionDetected::OneYuanOneTimeSolution(unsigned char& number, double coefficient[], unsigned char &m_iEquationSolutionNum, double result[])
{
	if (number >= 2 && coefficient[2] != 0)
	{
		result[0] = (-coefficient[1] / coefficient[2]);
		m_iEquationSolutionNum = 1;
		return true;
	}

	return false;
}

