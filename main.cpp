/* Copyright 2016 CyberTech Labs Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or ifstream.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * This file was modified by Yurii Litvinov to make it comply with the requirements of trikRuntime
 * project. See git revision history for detailed changes. */

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <QtGui/QApplication>
#include <QtCore/QDebug>

#include <trikControl/brickFactory.h>
#include <trikControl/brickInterface.h>

#include <QtCore/QTimer>

#include "accelerometerListener.h"

#define leftM brick->motor("M3")
#define rightM brick->motor("M4")

#define leftEnc  brick->encoder("E3")
#define rightEnc brick->encoder("E4")

#define sonarSens1 brick->sensor("A2")
#define sonarSens2 brick->sensor("A1")
#define servMotor brick->motor("S5")
	
#define sizeOfMap 100
#define printingSize 30
#define encC 750
#define encTurnC 45
#define baseSpeed 50

typedef float LD;

const LD PI = 3.1415926535897932384626433832795;
const LD p = 0.1;
const int eps = 15;
const int maxDX = 2;

using namespace std;

int slamMap[sizeOfMap][sizeOfMap];
int slamMask[sizeOfMap][sizeOfMap];

int currX;
int currY;
int currAngle;

void mysleep(double n)
{
	usleep(n * 1000000);
}

void clearMask(int ourX, int ourY, int size)
{
	for (int i = ourX - size; i <= ourX + size; i++)
	{
	    for (int j = ourY - size; j <= ourY + size; j++)
	    {
        	if(i >= 0 && i < sizeOfMap && j >= 0 && j < sizeOfMap)
			{
				slamMask[i][j] = 0;
			}
		}
	}
}

void cloneToMap(int ourX, int ourY, int size)
{
	for (int i = ourX - size; i <= ourX + size; i++)
	{
	    for (int j = ourY - size; j <= ourY + size; j++)
	    {
        	if(i >= 0 && i < sizeOfMap && j >= 0 && j < sizeOfMap)
			{
				//if(slamMap[i][j] != slamMask[i][j])
				slamMap[i][j] = slamMask[i][j];
			}
		}
	}
}

void buildMask(int ourX, int ourY, vector <int> distList, int startPos, int size) 
{
	int angle = -1;

	for(int i = startPos; i < startPos + 400; i++)
	{
		angle++;

		int pos = i % 400;
		LD alpha = LD(angle) / 200.0 * PI;
		LD dist = LD(distList[pos]);

		//cout << i << endl;

		LD dx = dist * cos(alpha);
		LD dy = dist * sin(alpha);

		if(dist > 10 && dist < 70)
		{
			if(ourX - int(dy/2.0) >= 0 && ourX - int(dy/2.0) < sizeOfMap && ourY + int(dx/2.0) >= 0 && ourY + int(dx/2.0) < sizeOfMap)
			{
				if(abs(dy/2.0) <= size && abs(dx/2.0) <= size)
					slamMask[ourX - int(dy/2.0)][ourY + int(dx/2.0)] = 1;
			}
		}
	}
}

vector <int> buildDistList(trikControl::BrickInterface *brick) 
{
  	vector <int> laserData(410);

	servMotor->setPower(-100);
	mysleep(0.5);

	for(int i = 0; i < 400; i++)
		laserData[i] = -1;

	for(int i = -100; i <= 100; i++)
	{
		servMotor->setPower(i);
		mysleep(0.05);

		int dist1 = sonarSens1->read();
		int dist2 = sonarSens2->read();

		if(dist1 > 10 && dist1 < 70 && i + 100 >= 0 && i + 100 < 400)
			laserData[i + 100] = dist1;
		else
		{
			if(i >= 0 && i < 400)
				laserData[i] = -1;
		}
		
		if(dist2 > 10 && dist2 < 70 && i + 300 >= 0 && i + 300 < 400)
			laserData[i + 300] = dist2;
		else
		{
			if(i + 300 >= 0 && i + 300 < 400)
				laserData[i + 300] = -1;	
		}
	}

	for(int i = 100; i >= -100; i--)
	{
		servMotor->setPower(i);
		mysleep(0.05);

		int dist1 = sonarSens1->read();
		int dist2 = sonarSens2->read();

		if(dist1 > 10 && dist1 < 70 && i + 100 >= 0 && i + 100 < 400 && laserData[i + 100] != -1)
			laserData[i + 100] = (laserData[i + 100] + dist1) / 2;

		if(dist2 > 10 && dist2 < 70 && i + 300 >= 0 && i + 300 < 400 && laserData[i + 300] != -1)
			laserData[i + 300] = (laserData[i + 300] + dist2) / 2;
	}
	
	servMotor->setPower(0);
	mysleep(0.5);

	vector <int> tmp(400);
	for(int i = 0; i < 400; i++)
	{
		//cout << i << ") " << laserData[i] << endl;
		tmp[i] = laserData[i];
	}

	for(int counter = 0; counter < 5; counter++)
	{
		for(int i = 1; i < 399; i++)
		{
			if(laserData[i] == -1)
				laserData[i - 1] = -1;
		}
		for(int i = 398; i >= 0; i--)
		{
			if(laserData[i] == -1)
				laserData[i + 1] = -1;
		}
	}

	//for(int i = 0; i < 400; i++)
	//	cout << i << ") " << tmp[i] << " " << laserData[i] << endl;

	return laserData;
}

void moveForvard(trikControl::BrickInterface *brick, int distance)
{
	//cout << "Go" << endl;
	
	leftEnc->reset();
	rightEnc->reset();

	rightM->setPower(0);
	leftM->setPower(0);
	
	LD l = 0;
	LD r = 0;
	
	//cout << "Go1" << endl;
	
	while((l + r) / 2 < distance * encC)
	{
		LD rSpeed = baseSpeed;
		LD lSpeed = baseSpeed;

		//cout << "Powers" << lSpeed << " " << rSpeed << endl;

		rightM->setPower(rSpeed);
		leftM->setPower(lSpeed);

		mysleep(0.05);	
		
		l = leftEnc->readRawData();
		r = -rightEnc->readRawData();

		//cout << "!" << l << " " << r << " : " << (l + r) / 2 << endl;
	}

	cout << "!" << l << " " << r << endl;

	rightM->setPower(0);
	leftM->setPower(0);
}

void turn(trikControl::BrickInterface *brick, int angle)
{
	leftEnc->reset();
	rightEnc->reset();

	rightM->setPower(0);
	leftM->setPower(0);
	
	LD l = 0;
	LD r = 0;
	
	while((l + r) / 2 < angle * encTurnC)
	{
		LD rSpeed = baseSpeed;
		LD lSpeed = baseSpeed;

		//cout << "Powers" << lSpeed << " " << rSpeed << endl;

		rightM->setPower(-rSpeed);
		leftM->setPower(lSpeed);

		mysleep(0.05);	
		
		l = leftEnc->readRawData();
		r = rightEnc->readRawData();

		//cerr << "!" << l << " " << r << " : " << (l + r) / 2 << " " << angle * encTurnC << endl;
	}

	//cout << "!" << l << " " << r << endl;

	rightM->setPower(0);
	leftM->setPower(0);
}

void printMask(int printZoneX, int printZoneY, int printZoneSize)
{
	for(int i = printZoneX - printZoneSize; i <= printZoneX + printZoneSize; i++)
	{
		for(int j = printZoneY - printZoneSize; j <= printZoneY + printZoneSize; j++)
		{
			if(i == currX && j == currY)
				cout << "R";
			else
			{
				if(slamMask[i][j] == 0)
					cout << '.';
				else
				{
					cout << '#';
				}
			}
		}
		cout << endl;
	}
	cout << "-------------------------" << endl << endl;
}

void printMap(int printZoneX, int printZoneY, int printZoneSize)
{
	for(int i = printZoneX - printZoneSize; i <= printZoneX + printZoneSize; i++)
	{
		for(int j = printZoneY - printZoneSize; j <= printZoneY + printZoneSize; j++)
		{
			if(i == currX && j == currY)
				cout << "R";
			else
			{
				if(slamMap[i][j] == 0)
					cout << '.';
				else
				{
					cout << '#';
				}
			}
		}
		cout << endl;
	}
	cout << "-------------------------" << endl << endl;
}

int countSame(int x, int y, int size)
{
	int answer = 0;

	for(int i =  x - size; i <= x + size; i++)
	{
		for(int j =  y - size; j <= y + size; j++)
		{
			if(slamMask[i][j] == 1 && slamMap[i][j] == 1)
				answer++;
			if(slamMask[i][j] == 0 && slamMap[i][j] == 0)
				answer++;
		}
	}
	return answer;
}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	freopen("log.txt", "w", stdout);

	trikControl::BrickInterface *brick = trikControl::BrickFactory::create(".", ".");
	brick->stop();
	
	currX = 50;
	currY = 50;

	vector <int> distList = buildDistList(brick);
	cerr << "Hello" << endl;
	buildMask(currX, currY, distList, 0, 50);

	cerr << "Builded" << endl;

	for (int i = 0; i < sizeOfMap; i++)
	{
	    for (int j = 0; j < sizeOfMap; j++)
	    {
        	slamMap[i][j] = slamMask[i][j];
        	//slamMask[i][j] = 0;
		}
	}
	clearMask(currX, currY, 50);

	printMap(currX, currY, printingSize);

	int moveLength = 0;

	while(slamMap[currX - moveLength][currY] != 1)
	{
		if(!(currX - moveLength >= 0 && currY - moveLength < sizeOfMap))
			break;

		moveLength++;
	}
	
	moveLength -= 3;

	cout << moveLength << endl;
	moveForvard(brick, moveLength);
	currX -= moveLength;

	distList = buildDistList(brick);
	cerr << "Hello" << endl;
	buildMask(currX, currY, distList, 0, eps);

	cout << endl << "Base mask" << endl;

	printMask(currX, currY, eps);
	clearMask(currX, currY, eps);

	int same = 0;
	int sameDX = 0;
	int sameDY = 0;
	int sameDAngle = 0;			

	//buildMask(currX, currY, distList, 100, 30);
	//cout << endl << endl;

	//printMask(currX, currY, printingSize);

	for(int dx = -2; dx <= 2; dx++)
	{	
		for(int dy = -2; dy <= 2; dy++)
		{
			cerr << dx << " " << dy << endl;
			for(int dAngle = 380; dAngle < 400; dAngle++)
			{
				buildMask(currX + dx, currY + dy, distList, dAngle, eps);
				
				int tempSame = countSame(currX, currY, eps);
				//cout << dx << " " << dy << " " << dAngle << " : " << tempSame << endl;
				clearMask(currX + dx, currY + dy, eps);

				if(tempSame > same)
				{
					same = tempSame;
					sameDX = dx;
					sameDY = dy;
					sameDAngle = dAngle;
				}
			}

			for(int dAngle = 0; dAngle <= 20; dAngle++)
			{
				buildMask(currX + dx, currY + dy, distList, dAngle, eps);	
				
				int tempSame = countSame(currX, currY, eps);
				//cout << dx << " " << dy << " " << dAngle << " : " << tempSame << endl;
				clearMask(currX + dx, currY + dy, eps);

				if(tempSame > same)
				{
					same = tempSame;
					sameDX = dx;
					sameDY = dy;
					sameDAngle = dAngle;
				}
			}
		}
	}

	cerr << "MARCHED" << endl;

	cout << sameDX << " " << sameDY << " " << sameDAngle << endl;

	currX -= sameDX;
	currY -= sameDY;
	currAngle = sameDAngle;
	
	buildMask(currX + sameDX, currY + sameDY, distList, sameDAngle, eps);	
	//printMask(currX, currY, eps);
	//printMap(currX, currY, eps);
	cloneToMap(currX, currY, eps);
	printMap(currX, currY, printingSize);
	clearMask(currX, currY, eps);

	int tmpAngle = currAngle;
	if(currAngle > 200)
		tmpAngle  = tmpAngle - 400;
	
	turn(brick, 150 + tmpAngle);
	moveForvard(brick, moveLength);

	currAngle += 150;
	currAngle %= 400;

	distList = buildDistList(brick);
	buildMask(currX, currY, distList, currAngle, eps);

	cout << "Last" << endl;
	printMask(currX, currY, eps);
	
	brick->stop();
	
	return a.exec();
}
