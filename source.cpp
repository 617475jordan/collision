#include <iostream>
#include <iomanip>
#include <math.h>
//#include "matrix.h"
#include <vector>
#include <algorithm>
#include <random>
#include "collisionDetected.h"
#include<windows.h>
#include <time.h>
using namespace std;


bool Cmp(Position firstPosition, Position secondPosition)
{
	if (firstPosition.x != secondPosition.x)
		return firstPosition.x < secondPosition.x;
	else
		return secondPosition.x < secondPosition.x;
}

int main()
{

	std::random_device rd;
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(-100.0, 100.0);
	std::uniform_real_distribution<> _dis(1, 16);
	unsigned long long count = 0;
	while (count < 1e6)
	{

		float m_time = clock();
#define VehicleNum 5
#define LevelCoefficient 4  //a*x*x*x*x+b*x*x*x+c*x*x+d*x+e

		std::vector<Position> m_firstVehiclePosition(VehicleNum);
		std::vector<Position> m_secondVehiclePosition(VehicleNum);




		/*m_firstVehiclePosition[0].y = 1;
		m_firstVehiclePosition[0].x = 0;
		m_firstVehiclePosition[1].y = 1;
		m_firstVehiclePosition[1].x = 2;
		m_firstVehiclePosition[2].y = 1;
		m_firstVehiclePosition[2].x = 3;
		m_firstVehiclePosition[3].y = 1;
		m_firstVehiclePosition[3].x = 4;
		m_firstVehiclePosition[4].y = 1;
		m_firstVehiclePosition[4].x = 6;*/

		for (int i = 0; i < VehicleNum; i++)
		{
			m_firstVehiclePosition[i].x = i;
			m_firstVehiclePosition[i].y = i*i;
			m_secondVehiclePosition[i].x = i;
			m_secondVehiclePosition[i].y = sqrt(i*1.0);
		}
		collisionDetected *m_firstCollisionDetected = new collisionDetected(LevelCoefficient, m_firstVehiclePosition);
		collisionDetected *m_secondCollisionDetected = new collisionDetected(LevelCoefficient, m_secondVehiclePosition);
		//std::sort(m_firstVehiclePosition.begin(), m_firstVehiclePosition.end(), Cmp);
		//std::sort(m_secondVehiclePosition.begin(), m_secondVehiclePosition.end(), Cmp);

		if (LevelCoefficient < 1)
		{
			return 0;
		}

		bool m_firstTraceIsJudgePositionYChange = m_firstCollisionDetected->IsTwoCoordinateConsistent(m_firstVehiclePosition, "col");
		bool m_secondTraceIsJudgePositionYChange = m_secondCollisionDetected->IsTwoCoordinateConsistent(m_secondVehiclePosition, "col");
		if (m_firstTraceIsJudgePositionYChange == false && m_secondTraceIsJudgePositionYChange == false)//������б�ʶ�Ϊ������ʱ������Ҫ���⴦��
		{
			float m_initialPositionY = m_firstVehiclePosition[0].y - m_secondVehiclePosition[0].y;
			float m_finalPositionY = m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].y - m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].y;
			if (m_initialPositionY*m_finalPositionY <= 0)
			{
				std::cout << "meeting" << std::endl;
			}
			else
			{
				std::cout << "not meeting" << std::endl;
			}
			return -1;
		}

		bool m_firstTraceIsJudgePositionXChange = m_firstCollisionDetected->IsTwoCoordinateConsistent(m_firstVehiclePosition, "row");
		bool m_secondTraceIsJudgePositionXChange = m_secondCollisionDetected->IsTwoCoordinateConsistent(m_secondVehiclePosition, "row");
		if (m_firstTraceIsJudgePositionXChange == false && m_firstTraceIsJudgePositionXChange == false)//������б�ʶ�Ϊ������ʱ������Ҫ���⴦��
		{
			float m_initialPositionX = m_firstVehiclePosition[0].x - m_secondVehiclePosition[0].x;
			float m_finalPositionX = m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x - m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].x;
			if (m_initialPositionX*m_finalPositionX <= 0)
			{
				std::cout << "meeting" << std::endl;
			}
			else
			{
				std::cout << "not meeting" << std::endl;
			}
			return -1;
		}

		float m_fSuccessPoint[2] = { 0 };
		unsigned int m_iSuccessNum = 0;

		float m_fFirstVehiclePolynomialCoefficient[10] = { 0 }; //�������Ķ���ʽ��ϵ��
		float m_fSecondVehiclePolynomialCoefficient[10] = { 0 }; //�������Ķ���ʽ��ϵ��
		float m_fPolynomialCoefficient[10] = { 0 }; //�������Ķ���ʽ��ϵ��

		if (m_firstTraceIsJudgePositionYChange == false && m_secondTraceIsJudgePositionYChange == true)//consdier that position which has been sorted,where the min position may be the first position
		{
			if (m_firstVehiclePosition[0].x >= m_secondVehiclePosition[0].x&&m_firstVehiclePosition[0].x <= m_secondVehiclePosition[0].x)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else if (m_firstTraceIsJudgePositionYChange == true && m_secondTraceIsJudgePositionYChange == false)
		{
			if (m_secondVehiclePosition[0].x >= m_firstVehiclePosition[0].x&&m_secondVehiclePosition[0].x <= m_firstVehiclePosition[0].x)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
			/*else if (m_firstTraceIsJudgePositionXChange == true && m_firstTraceIsJudgePositionYChange == true &&
				m_secondTraceIsJudgePositionXChange == true && m_secondTraceIsJudgePositionYChange == true)*/
		{

			std::vector<Position> m_cmpVehiclePosition(VehicleNum);


			float m_firstSpeend = _dis(gen);
			float m_secondSpeend = _dis(gen);
			unsigned int m_positionSize = m_firstVehiclePosition.size();
			//float m_fFirstVehiclePolynomialCoefficient[10] = { 0 }; //�������Ķ���ʽ��ϵ��
			m_firstCollisionDetected->AugmentedMatrixAssignment(m_positionSize, LevelCoefficient, m_fFirstVehiclePolynomialCoefficient);

			//float m_fSecondVehiclePolynomialCoefficient[10] = { 0 }; //�������Ķ���ʽ��ϵ��
			m_secondCollisionDetected->AugmentedMatrixAssignment(m_positionSize, LevelCoefficient, m_fSecondVehiclePolynomialCoefficient);


			collisionDetected *m_cmpCollisionDetected = new collisionDetected(LevelCoefficient, m_cmpVehiclePosition);
			//float m_fPolynomialCoefficient[10] = { 0 }; //�������Ķ���ʽ��ϵ��
			m_cmpVehiclePosition.clear();
			//m_cmpCollisionDetected->AugmentedMatrixAssignment(m_positionSize, LevelCoefficient,m_fPolynomialCoefficient);
			for (unsigned int i = 0; i < 10; i++)
			{
				m_fPolynomialCoefficient[i] = m_fFirstVehiclePolynomialCoefficient[i] - m_fSecondVehiclePolynomialCoefficient[i];
			}

			float m_fResult[2] = { 0 };
			bool m_bMeetPosition = m_cmpCollisionDetected->Derivation(LevelCoefficient, m_fPolynomialCoefficient, m_fResult);//�������������ó������ߣ��������߶��Ƿ���������

			m_cmpCollisionDetected = NULL;
			delete[] m_cmpCollisionDetected;

			if (m_bMeetPosition == true)
			{
				float m_fBias = 2;//consider the vehicle width and height which cases the time cost
				for (unsigned int i = 0; i < 2; i++)
				{
					if (m_firstVehiclePosition[0].x > +m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x)
					{
						if (m_fResult[i] <= m_firstVehiclePosition[0].x&&m_fResult[i] >= m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x)
						{

							float m_firstVehicleTime = abs(m_firstCollisionDetected->GetLineLength(LevelCoefficient, m_fFirstVehiclePolynomialCoefficient,
								m_firstVehiclePosition[0].x, m_fResult[i])) / m_firstSpeend;
							float m_secondVehicleTime = abs(m_secondCollisionDetected->GetLineLength(LevelCoefficient, m_fSecondVehiclePolynomialCoefficient,
								m_secondVehiclePosition[0].x, m_fResult[i])) / m_secondSpeend;

							if (abs(m_firstVehicleTime - m_secondVehicleTime) <= m_fBias)
							{
								/*std::cout << "Distance:" << abs(m_firstVehicleTime - m_secondVehicleTime) << std::endl;
								std::cout << "time:" << m_firstVehicleTime << " " << m_secondVehicleTime << std::endl;
								std::cout << "meeting point:" << m_fResult[i] << std::endl;
								std::cout << "success" << std::endl;*/
								m_fSuccessPoint[m_iSuccessNum] = m_fResult[i];
								m_iSuccessNum++;
							}
						}
						else
						{
						//	std::cout << "failed" << std::endl;
						}
					}
					else
					{
						if (m_fResult[i] >= m_firstVehiclePosition[0].x&&m_fResult[i] <= m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x)
						{
							float m_firstVehicleTime = abs(m_firstCollisionDetected->GetLineLength(LevelCoefficient, m_fFirstVehiclePolynomialCoefficient,
								m_firstVehiclePosition[0].x, m_fResult[i])) / m_firstSpeend;
							float m_secondVehicleTime = abs(m_secondCollisionDetected->GetLineLength(LevelCoefficient, m_fSecondVehiclePolynomialCoefficient,
								m_secondVehiclePosition[0].x, m_fResult[i])) / m_secondSpeend;

							if (abs(m_firstVehicleTime - m_secondVehicleTime) <= m_fBias)
							{
								/*std::cout << "Distance:" << abs(m_firstVehicleTime - m_secondVehicleTime) << std::endl;
								std::cout << "time:" << m_firstVehicleTime << " " << m_secondVehicleTime << std::endl;
								std::cout << "meeting point:" << m_fResult[i] << std::endl;
								std::cout << "success" << std::endl;*/
								m_fSuccessPoint[m_iSuccessNum] = m_fResult[i];
								m_iSuccessNum++;
							}
						}
						else
						{
							//std::cout << "failed" << std::endl;
						}
					}
				}
			}
		}
		m_firstCollisionDetected = NULL;
		delete[]m_firstCollisionDetected;
		m_secondCollisionDetected = NULL;
		delete[] m_secondCollisionDetected;
		for (unsigned int i = 0; i < m_iSuccessNum; i++)
		{
			std::cout << m_fSuccessPoint[i] << std::endl;
		}
		if (m_iSuccessNum > 0)
		{
			for (unsigned int i = 1; i <= 4; i++)
			{
				std::cout << "(" << m_fFirstVehiclePolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")+";
			}
			std::cout << std::endl;
			for (unsigned int i = 1; i <= 4; i++)
			{
				std::cout << "(" << m_fSecondVehiclePolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")+";
			}
			std::cout << std::endl;
			for (unsigned int i = 1; i <= 4; i++)
			{
				std::cout << "(" << m_fPolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")+";
			}
			std::cout << std::endl;
			std::cout << std::endl;
			std::cout << std::endl;
			for (unsigned int i = 0; i < VehicleNum; i++)
			{
				std::cout << "(" << m_firstVehiclePosition[i].x << "," << m_firstVehiclePosition[i].y << ")";
			}
			std::cout << std::endl;
			for (unsigned int i = 0; i < VehicleNum; i++)
			{
				std::cout << "(" << m_secondVehiclePosition[i].x << "," << m_secondVehiclePosition[i].y << ")";
			}
			std::cout << std::endl;
			//Sleep(60000);
		}
		m_firstVehiclePosition.clear();
		m_secondVehiclePosition.clear();
		count++;
		std::cout << clock() - m_time << "ms" << std::endl;

		//system("pause");
	}

	return 0;
}
