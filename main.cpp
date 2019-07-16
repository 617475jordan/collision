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
#define PI 3.1415926535898


// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// using namespace cv;
// #if _DEBUG
// #pragma  comment(lib,"..\\lib\\opencv_world340d.lib")
// #else
// #pragma  comment(lib,"..\\lib\\opencv_world340.lib")
// #endif
bool Cmp(Position firstPosition, Position secondPosition)
{
    if (firstPosition.x != secondPosition.x)
        return firstPosition.x < secondPosition.x;
    else
        return secondPosition.x < secondPosition.x;
}

int main()
{
    double theta = 0;

    std::random_device rd;
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-100.0, 100.0);
    std::uniform_real_distribution<> _dis(-10, 10);
    std::uniform_real_distribution<> speed(0, 16);
    unsigned long long count = 0;
    clock_t m_time = clock();
    unsigned long totalNum = 100000;
    while (count < totalNum)
    {


#define VehiclePositionPointNum 10
        unsigned char  LevelCoefficient = 4;  //a*x*x*x*x+b*x*x*x+c*x*x+d*x+e

        std::vector<Position> m_firstVehiclePosition(VehiclePositionPointNum);
        std::vector<Position> m_secondVehiclePosition(VehiclePositionPointNum);


        //m_firstVehiclePosition[0].y = 0;
        //m_firstVehiclePosition[0].x = 0;
        //m_firstVehiclePosition[1].y = 1;
        //m_firstVehiclePosition[1].x = 1;
        //m_firstVehiclePosition[2].y = 4;
        //m_firstVehiclePosition[2].x = 2;
        //m_firstVehiclePosition[3].y = 9;
        //m_firstVehiclePosition[3].x = 3;
        //m_firstVehiclePosition[4].y = 16;
        //m_firstVehiclePosition[4].x = 4;


        //m_secondVehiclePosition[0].y = 2;
        //m_secondVehiclePosition[0].x = 0;
        //m_secondVehiclePosition[1].y = 1;
        //m_secondVehiclePosition[1].x = 1;
        //m_secondVehiclePosition[2].y = -2;
        //m_secondVehiclePosition[2].x = 2;
        //m_secondVehiclePosition[3].y = -7;
        //m_secondVehiclePosition[3].x = 3;
        //m_secondVehiclePosition[4].y = -14;
        //m_secondVehiclePosition[4].x = 4;


        /*m_firstVehiclePosition[0].y = -1;
        m_firstVehiclePosition[0].x = 0;
        m_firstVehiclePosition[1].y = 0;
        m_firstVehiclePosition[1].x = 1;
        m_firstVehiclePosition[2].y = 7;
        m_firstVehiclePosition[2].x = 2;
        m_firstVehiclePosition[3].y = 32;
        m_firstVehiclePosition[3].x = 3;
        m_firstVehiclePosition[4].y = 87;
        m_firstVehiclePosition[4].x = 4;*/


        /*m_secondVehiclePosition[0].y = 1;
        m_secondVehiclePosition[0].x = 0;
        m_secondVehiclePosition[1].y = -4;
        m_secondVehiclePosition[1].x = 1;
        m_secondVehiclePosition[2].y = 3;
        m_secondVehiclePosition[2].x = 2;
        m_secondVehiclePosition[3].y = 46;
        m_secondVehiclePosition[3].x = 3;
        m_secondVehiclePosition[4].y = 149;
        m_secondVehiclePosition[4].x = 4;*/
        for (unsigned int i = 0; i < VehiclePositionPointNum; i++)
        {
            m_firstVehiclePosition[i].width = 10;
            m_firstVehiclePosition[i].height = 10;
            m_firstVehiclePosition[i].length = 10;
            m_firstVehiclePosition[i].speed = speed(gen);

            m_secondVehiclePosition[i].width = 8;
            m_secondVehiclePosition[i].height = 8;
            m_secondVehiclePosition[i].length = 8;
            m_secondVehiclePosition[i].speed = speed(gen);
            m_secondVehiclePosition[i].x = _dis(gen);
            m_secondVehiclePosition[i].y = dis(gen);

            m_firstVehiclePosition[i].x = _dis(gen);
            m_firstVehiclePosition[i].y = dis(gen);
        }

        sort(m_secondVehiclePosition.begin(), m_secondVehiclePosition.end(), Cmp);
        sort(m_firstVehiclePosition.begin(), m_firstVehiclePosition.end(), Cmp);
        /*for (int i = 0; i < VehicleNum; i++)
        {
        m_firstVehiclePosition[i].x = i;
        m_firstVehiclePosition[i].y = i*i;
        m_secondVehiclePosition[i].x = i;
        m_secondVehiclePosition[i].y = sqrt(i*1.0);
        }*/
        collisionDetected *m_firstCollisionDetected = new collisionDetected(/*LevelCoefficient,*/ m_firstVehiclePosition);
        collisionDetected *m_secondCollisionDetected = new collisionDetected(/*LevelCoefficient,*/ m_secondVehiclePosition);
        //std::sort(m_firstVehiclePosition.begin(), m_firstVehiclePosition.end(), Cmp);
        //std::sort(m_secondVehiclePosition.begin(), m_secondVehiclePosition.end(), Cmp);

        if (LevelCoefficient <= 1)
        {
            return 0;
        }

        bool m_firstTraceIsJudgePositionYChange = m_firstCollisionDetected->IsTwoCoordinateConsistent(m_firstVehiclePosition, "col");
        bool m_secondTraceIsJudgePositionYChange = m_secondCollisionDetected->IsTwoCoordinateConsistent(m_secondVehiclePosition, "col");
        if (m_firstTraceIsJudgePositionYChange == false && m_secondTraceIsJudgePositionYChange == false)//??????б????????????????????????
        {
            double m_initialPositionY = m_firstVehiclePosition[0].y - m_secondVehiclePosition[0].y;
            double m_finalPositionY = m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].y - m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].y;
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
        if (m_firstTraceIsJudgePositionXChange == false && m_firstTraceIsJudgePositionXChange == false)//??????б????????????????????????
        {
            double m_initialPositionX = m_firstVehiclePosition[0].x - m_secondVehiclePosition[0].x;
            double m_finalPositionX = m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x - m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].x;
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

        double m_fSuccessPoint[3] = { 0 };
        unsigned int m_iSuccessNum = 0;

        double m_fFirstVehiclePolynomialCoefficient[10] = { 0 }; //?????????????????
        double m_fSecondVehiclePolynomialCoefficient[10] = { 0 }; //?????????????????
        double m_fPolynomialCoefficient[10] = { 0 }; //?????????????????

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

            std::vector<Position> m_cmpVehiclePosition(VehiclePositionPointNum);



            size_t m_positionSize = m_firstVehiclePosition.size();
            //double m_fFirstVehiclePolynomialCoefficient[10] = { 0 }; //?????????????????
            m_firstCollisionDetected->AugmentedMatrixAssignment((int)m_positionSize, LevelCoefficient, m_fFirstVehiclePolynomialCoefficient);

            //double m_fSecondVehiclePolynomialCoefficient[10] = { 0 }; //?????????????????
            m_secondCollisionDetected->AugmentedMatrixAssignment((int)m_positionSize, LevelCoefficient, m_fSecondVehiclePolynomialCoefficient);


            collisionDetected *m_cmpCollisionDetected = new collisionDetected(/*LevelCoefficient,*/ m_cmpVehiclePosition);
            //double m_fPolynomialCoefficient[10] = { 0 }; //?????????????????
            m_cmpVehiclePosition.clear();
            //m_cmpCollisionDetected->AugmentedMatrixAssignment(m_positionSize, LevelCoefficient,m_fPolynomialCoefficient);
            for (unsigned int i = 0; i < 10; i++)
            {
                m_fPolynomialCoefficient[i] = m_fFirstVehiclePolynomialCoefficient[i] - m_fSecondVehiclePolynomialCoefficient[i];
            }

            double m_fResult[3] = { 0 };
            unsigned char m_iEquationSolutionNum = 0;
            bool m_bMeetPointFlag = m_cmpCollisionDetected->EquationSolution(LevelCoefficient, m_fPolynomialCoefficient, m_iEquationSolutionNum, m_fResult);//?????????????ó????????????????????????????

            m_cmpCollisionDetected = NULL;
            delete[] m_cmpCollisionDetected;

            if (m_bMeetPointFlag == true && m_iEquationSolutionNum > 0)
            {
                //double m_fBias = 2;//consider the vehicle width and height which cases the time cost
                for (unsigned char i = 0; i < (unsigned char)m_iEquationSolutionNum; i++)
                {

                    double m_fFirstVehicleMinPosition = (m_firstVehiclePosition[0].x < m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x) ?
                                                        m_firstVehiclePosition[0].x : m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x;
                    double m_fFirstVehicleMaxPosition = (m_firstVehiclePosition[0].x > m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x) ?
                                                        m_firstVehiclePosition[0].x : m_firstVehiclePosition[m_firstVehiclePosition.size() - 1].x;
                    double m_fSecondVehicleMinPosition = (m_secondVehiclePosition[0].x < m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].x) ?
                                                         m_secondVehiclePosition[0].x : m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].x;
                    double m_fSecondVehicleMaxPosition = (m_secondVehiclePosition[0].x > m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].x) ?
                                                         m_secondVehiclePosition[0].x : m_secondVehiclePosition[m_secondVehiclePosition.size() - 1].x;


                    double m_fMinPosition = (m_fFirstVehicleMinPosition > m_fSecondVehicleMinPosition) ? m_fFirstVehicleMinPosition : m_fSecondVehicleMinPosition;
                    double m_fMaxPosition = (m_fFirstVehicleMaxPosition < m_fSecondVehicleMaxPosition) ? m_fFirstVehicleMaxPosition : m_fSecondVehicleMaxPosition;


                    if (m_fMinPosition <= m_fResult[i] && m_fMaxPosition >= m_fResult[i])
                    {

                        double m_firstVehicleTime = abs(m_firstCollisionDetected->GetLineLength(LevelCoefficient, m_fFirstVehiclePolynomialCoefficient,
                                                                                                m_firstVehiclePosition[0].x, m_fResult[i])) / m_firstVehiclePosition[0].speed;
                        double m_secondVehicleTime = abs(m_secondCollisionDetected->GetLineLength(LevelCoefficient, m_fSecondVehiclePolynomialCoefficient,
                                                                                                  m_secondVehiclePosition[0].x, m_fResult[i])) / m_secondVehiclePosition[0].speed;


                        double m_fTimeBias = abs(m_firstVehicleTime - m_secondVehicleTime);
                        if (m_fTimeBias == 0)
                        {
                            m_fSuccessPoint[m_iSuccessNum] = m_fResult[i];
                            m_iSuccessNum++;
                            std::cout << "collision" << " " << m_fMinPosition << " " << m_fMaxPosition << std::endl;
                            break;
                        }
                        else if (m_fTimeBias > 0)
                        {
                            double  m_firstCofficientDerivation = 0;
                            bool m_firstDerivationFlag = m_firstCollisionDetected->Derivation(LevelCoefficient,
                                                                                              m_fFirstVehiclePolynomialCoefficient, m_fResult[i], m_firstCofficientDerivation);
                            double  m_secondCofficientDerivation = 0;
                            bool m_secondDerivationFlag = m_firstCollisionDetected->Derivation(LevelCoefficient,
                                                                                               m_fSecondVehiclePolynomialCoefficient, m_fResult[i], m_secondCofficientDerivation);

                            if (m_firstDerivationFlag == true && m_secondDerivationFlag == true)
                            {
                                double  m_fTheta = abs(atan(m_firstCofficientDerivation - m_secondCofficientDerivation));
                                if (m_firstVehicleTime < m_secondVehicleTime)
                                {
                                    double m_fDistanceBias = m_firstVehiclePosition[0].speed*sin(m_fTheta)*m_fTimeBias;
                                    if (m_fDistanceBias >= (m_firstVehiclePosition[0].length + m_secondVehiclePosition[0].width) / 2)
                                    {
                                        //	std::cout << "safe" << std::endl;
                                        //break;
                                    }
                                    else
                                    {
                                        std::cout << "collision" << " " << m_fMinPosition << " " << m_fMaxPosition << std::endl;
                                        m_fSuccessPoint[m_iSuccessNum] = m_fResult[i];
                                        m_iSuccessNum++;
                                        //break;
                                    }
                                }
                                else
                                {
                                    double m_fDistanceBias = m_secondVehiclePosition[0].speed*sin(m_fTheta)*m_fTimeBias;
                                    if (m_fDistanceBias >= (m_secondVehiclePosition[0].length + m_firstVehiclePosition[0].width) / 2)
                                    {
                                        //	std::cout << "safe" << std::endl;

                                        //break;
                                    }
                                    else
                                    {
                                        std::cout << "collision" << " " << m_fMinPosition << " " << m_fMaxPosition << std::endl;
                                        m_fSuccessPoint[m_iSuccessNum] = m_fResult[i];
                                        m_iSuccessNum++;
                                        //break;
                                    }
                                }

                            }

                            //if (abs(m_firstVehicleTime - m_secondVehicleTime) <= m_fBias)
                            //{
                            //	/*std::cout << "Distance:" << abs(m_firstVehicleTime - m_secondVehicleTime) << std::endl;
                            //	std::cout << "time:" << m_firstVehicleTime << " " << m_secondVehicleTime << std::endl;
                            //	std::cout << "meeting point:" << m_fResult[i] << std::endl;
                            //	std::cout << "success" << std::endl;*/
                            //	m_fSuccessPoint[m_iSuccessNum] = m_fResult[i];
                            //	m_iSuccessNum++;
                            //}
                        }
                    }
                }
            }
        }
        m_firstCollisionDetected = NULL;
        delete[]m_firstCollisionDetected;
        m_secondCollisionDetected = NULL;
        delete[] m_secondCollisionDetected;

        std::cout << "num:" << count << std::endl;
        for (unsigned int i = 0; i < m_iSuccessNum; i++)
        {
            std::cout << m_fSuccessPoint[i] << std::endl;
        }
        if (m_iSuccessNum > 0)
        {
            for (unsigned int i = 1; i <= 4; i++)
            {
                if (i < 4)
                {
                    std::cout << "(" << m_fFirstVehiclePolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")+";
                }
                else
                {
                    std::cout << "(" << m_fFirstVehiclePolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")" << std::endl;
                }

            }
            for (unsigned int i = 1; i <= 4; i++)
            {
                if (i < 4)
                {
                    std::cout << "(" << m_fSecondVehiclePolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")+";
                }
                else
                {
                    std::cout << "(" << m_fSecondVehiclePolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")" << std::endl;
                }
            }
            for (unsigned int i = 1; i <= 4; i++)
            {
                if (i < 4)
                {
                    std::cout << "(" << m_fPolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")+";
                }
                else
                {
                    std::cout << "(" << m_fPolynomialCoefficient[i] << ")*pow(x0," << i - 1 << ")" << std::endl << std::endl;
                }
            }
            /*	std::cout << std::endl;
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
            }*/
            //Sleep(60000);
        }
        m_firstVehiclePosition.clear();
        m_secondVehiclePosition.clear();
        count++;

    }
    std::cout << (clock() - m_time) / totalNum << "ms" << std::endl;

    system("pause");
    return 0;
}
