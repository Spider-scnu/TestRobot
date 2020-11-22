#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <ctime>
#include <string>
#include <thread>
#include <math.h>
#include <algorithm>
#include <queue>
#include <map>
#include <set>
#include <mutex>

using namespace std;

bool RecordLog(string s);


//��ͼ����Ķ���
class Point{         // the point in the map
public:
	Point() :x(0), y(0){}
	Point(int tx, int ty){
		x = tx;
		y = ty;
	}
	bool operator==(const Point& r) const {
		return (x == r.x) && (y == r.y);
	}
	bool operator < (const Point &r) const {
		int a = x * x + y * y;
		int b = r.x * r.x + r.y * r.y;
		if (a < b){
			return true;
		}
		else if (a == b){
			if (x < r.x)
				return true;
			else
				return false;
		}
		else{
			return false;
		}
	}
	Point up(){
		return Point(x - 1 , y);
	}
	Point down(){
		return Point(x + 1 , y);
	}
	Point left(){
		return Point(x , y - 1);
	}
	Point right(){
		return Point(x , y + 1);
	}
	friend class Rule;
	friend class Robot;
	friend class Task;
	friend int main();
private:
	int x;
	int y;
};

//Ȩ�ص���Ķ���
class WeightPoint{
private:
	int up;
	int down;
	int left;
	int right;
public:
	WeightPoint() :up(0), down(0), left(0), right(0){}
	bool operator == (const WeightPoint& r) const {
		return (up == r.up) && (down == r.down) && (left == r.left) && (right == r.right);
	}
	friend class Rule;
	friend class Robot;
	friend int main();
};

//������Ķ���
class Task{            // class task to describe the details of the task
public:
	Task():id(0), publishTime(0),waitTime(0), source(), target(){}
	bool operator == (const Task& r) const {
		return (id == r.id) &&
			(publishTime == r.publishTime) &&
			(waitTime == r.waitTime) &&
			(source == r.source) &&
			(target == r.target);
	}
	friend class Rule;
	friend class Robot;
	friend int main();
private:
	int id;            // task id
	int publishTime;   // the time generating task
	int waitTime;      // the time next task wait
	Point source;      // starting point of task
	Point target;      // end point of task
};

//������Ķ���
class Rule{
private:
	int mapRowNumber;                      // the number of map row
	int mapColumnNumber;                   // the number of map column
	int robotNumber;                       // the number of robot
	int globalTime;                        // the time of system
	int taskNum;                           // the number of taskNum
	int taskId;                            // taskId is used to record the id of task
	vector<float> bidTable;                // the talbe of all robot's bid price
	vector<vector<int>> map;               // use two dimension to record the map
	vector<Point> robotInitPosition;       // the init position of robot
	vector<Point> robotCurrentPosition;    // the current position of robot
	vector<Point> robotTargetPosition;     // the target position of robot
	vector<Task> ToDoTask, DoingTask, DoneTask; // threee vector of task
	vector<Task> waitToAssigned;             // �ȴ�������
	vector<vector<WeightPoint>> weightMap;   // the weight of map
public:
	Rule() :mapRowNumber(0),mapColumnNumber(0),globalTime(0),robotNumber(0),taskNum(0),
		taskId(0){cout << "Construct a Object of Class Rule " << endl; RecordLog("Construct a Object of Class Rule ");}
	inline int getMapRowNumber(){return mapRowNumber;}
	inline int getMapColumnNumber(){return mapColumnNumber;}
	inline int getRobotNumber(){return robotNumber;}
	inline bool setGlobalTime(int newGlobalTime){globalTime = newGlobalTime;return true;}
	inline int getGlobalTime(){return globalTime;}
	vector<vector<int>> getMap(){return map;}
	vector<vector<WeightPoint>> getWeightMap(){ return weightMap; }
	Point getRobotInitPosition(int RobotNumber){return robotInitPosition[RobotNumber];}
	bool setRobotCurrentPosition(int RobotNumber, Point Position){
		robotCurrentPosition[RobotNumber] = Position;
		return true;
	}
	bool setRobotTargetPosition(int RobotNumber, Point Position){
		robotTargetPosition[RobotNumber] = Position;
		return true;
	}
	bool ReadMap();
	bool GlobalTime();
	bool ReadRobotInitPosition();
	bool RecordCurrentAndTargetPosition();
	bool GenerateTask();
	bool ReadTask();
	int ShortDistance(Point& source, Point& target);
	bool AssignTask();
	friend int main();
};

bool 
Rule::ReadMap(){    // Read the map from given file and form the weight map
	ifstream f;
	f.open("../TestRobot/InitMap.txt", ifstream::in);
	if (f){
		f >> mapRowNumber;
		char c;
		f >> c;
		f >> mapColumnNumber;
		for (int r = 0; r < mapRowNumber; ++r){
			vector<int> oneRow;
			for (int c = 0; c < mapColumnNumber; ++c){
				int onepoint;
				char t;
				f >> onepoint;
				oneRow.push_back(onepoint);
				if (c != (mapColumnNumber - 1))
					f >> t;
			}
			map.push_back(oneRow);
		}
	}
	else{
		cout << " Failed to open the InitMap.txt! " << endl;
		RecordLog("Failed to open the InitMap.txt!");
		return false;
	}
	f.close();
	//mapRowNumber = mapRowNumber - 7;
	for (int i = 0; i < mapRowNumber; ++i){
		vector<WeightPoint> oneRow;
		for (int j = 0; j < mapColumnNumber; ++j){
			WeightPoint t;
			if (map[i][j] == 1){
				t.up = -1;
				t.down = -1;
				t.left = -1;
				t.right = -1;
			}
			else{
				if ((i - 1) >= 0 && (i - 1) < mapRowNumber){
					if (map[i - 1][j] == 0)
						t.up = 1;
					else
						t.up = -1;
				}
				else
					t.up = -1;
				if ((i + 1) >= 0 && (i + 1) < mapRowNumber){
					if (map[i + 1][j] == 0)
						t.down = 1;
					else
						t.down = -1;
				}
				else
					t.down = -1;
				if ((j - 1) >= 0 && (j - 1) < mapColumnNumber){
					if (map[i][j - 1] == 0)
						t.left = 1;
					else
						t.left = -1;
				}
				else
					t.left = -1;
				if ((j + 1) >= 0 && (j + 1) < mapColumnNumber){
					if (map[i][j + 1] == 0)
						t.right = 1;
					else
						t.right = -1;
				}
				else
					t.right = -1;
			}
			oneRow.push_back(t);
		}
		weightMap.push_back(oneRow);
	}
	cout << "Success to read the map from file InitMap.txt" << endl;
	RecordLog("Success to read the map from file InitMap.txt");
	return true;
}

bool 
Rule::GlobalTime(){   // record the globaltime and print the global time
	++globalTime;
	cout << "GlobalTime is " << globalTime << endl;
	RecordLog("GlobalTime is "+to_string(globalTime));
	return true;
}

bool 
Rule::RecordCurrentAndTargetPosition(){
	ofstream f;
	f.open("../TestRobot/Robot_Current_Position.txt", ofstream::out);
	if (f){
		f << robotCurrentPosition.size() << endl;
		RecordLog("RecordCurrentAndTargetPosition:");
		RecordLog("RobotId   CurrentPosition   TargetPosition");
		for (int i = 1; i <= robotCurrentPosition.size(); ++i){
			f << i << "," << robotCurrentPosition[i - 1].x + 1 << "," << robotCurrentPosition[i - 1].y + 1 << ","
				<< robotTargetPosition[i - 1].x + 1 << "," << robotTargetPosition[i - 1].y + 1 << endl;
			RecordLog("   " + to_string(i) + "          [" + to_string(robotCurrentPosition[i - 1].x + 1) + ","
				+ to_string(robotCurrentPosition[i - 1].y + 1) + "]              [" + to_string(robotTargetPosition[i - 1].x + 1) +
				"," + to_string(robotTargetPosition[i - 1].y + 1)+"]");
		}
	}
	else
		return false;
	f.close();
	return true;
}

bool 
Rule::ReadRobotInitPosition(){
	ifstream f;
	f.open("../TestRobot/Robot_Init_Position.txt", ifstream::in);
	if (f){
		f >> robotNumber;
		bidTable.resize(robotNumber);
		Point target = Point(-1,-1);
		for (int i = 0; i < robotNumber; ++i){
			int x, y;
			int tempdata;
			char tempchar;
			f >> tempdata;
			f >> tempchar;
			f >> tempdata;
			x = tempdata-1;
			f >> tempchar;
			f >> tempdata;
			y = tempdata - 1;
			Point t;
			t.x = x;
			t.y = y;
			robotInitPosition.push_back(t);
			robotTargetPosition.push_back(target);
		}
		robotCurrentPosition = robotInitPosition;  // the init of robotCurrentPosition is robotInitPosition
		cout << "Success to read the robotInitPosition from file Robot_Init_Position.txt" << endl;
		RecordLog("Success to read the robotInitPosition from file Robot_Init_Position.txt");
	}
	else{
		return false;
	}
	f.close();
	return true;
}

bool 
Rule::GenerateTask(){
	// ����������
	cout << "Please input the taskNum:" << endl;
	cin >> taskNum;
	ofstream f;
	int ptime = 0;
	f.open("../TestRobot/Task.txt", ofstream::out);
	if (f){
		f << taskNum << endl;
		for (int i = 1; i <= taskNum; ++i){
			Task tempTask;
			tempTask.id = taskId + 1;
			++ taskId;
			tempTask.publishTime = ptime;
			tempTask.waitTime = rand() % 3 + 2;
			do{
				tempTask.source.x = rand() % (mapRowNumber - 1) + 1;
				tempTask.source.y = rand() % mapColumnNumber;
				tempTask.target.x = rand() % (mapRowNumber - 1) + 1;
				tempTask.target.y = rand() % mapColumnNumber;
			} while (tempTask.source == tempTask.target ||
				map[tempTask.source.x][tempTask.source.y] == 1 ||
				map[tempTask.target.x][tempTask.target.y] == 1);
			ptime += tempTask.waitTime;
			f << tempTask.id << " " << tempTask.publishTime << " " <<
				tempTask.waitTime << " " << tempTask.source.x << " " <<
				tempTask.source.y << " " << tempTask.target.x << " " <<
				tempTask.target.y << endl;
		}
	}
	else
		cout << "Failed to open the target file . Please check the filename . " << endl;
	f.close();
	cout << "Success to generate task" << endl;
	return true;
}

bool 
Rule::ReadTask(){
	ifstream f;
	f.open("../TestRobot/Task.txt", ifstream::in);
	if (f){
		f >> taskNum;
		Task tempTask;
		for (int i = 0; i < taskNum; ++i){
			f >> tempTask.id >> tempTask.publishTime >> tempTask.waitTime
				>> tempTask.source.x >> tempTask.source.y
				>> tempTask.target.x >> tempTask.target.y;
			ToDoTask.push_back(tempTask);
		}
	}
	else
		cout << "Failed to open the target file . Please check the filename ." << endl;
	f.close();
	cout << "Finish to read task" << endl;
	RecordLog("Finish to read task");
	return true;
}

//�������ඨ��
class Robot{
private:
	static int robotNumber;                 //����������
	static map<Point, int> robotPosition;   // ������λ��
	static vector<float> bidTable;   // �����˾��۱�
	static mutex mu;                 // ��ͼ��
	int Id;                   // robot Id , start from zero
	int Status;               // robot status,1 means to go to source of task , 2 means to go to end of task
	bool isBack;              // robot is back to init position
	int waitTime;             // robot waitTime
	float bidPrice;           // the price robot bid for task
	int mapRowNumber;         // the row number of map
	int mapColumnNumber;      // the column number of map
	int distance;             // �������ƶ�����
	int fightTime;              // �����˵�ǰ����ƶ�����
	int alreadyWait;            // �����˳�ͻ�ȴ�ʱ��
	Point initPosition;       // initPosition of robot
	Point nextPosition;        // ������֮ǰ��λ��
	Point currentPosition;    // currentPosition of robot
	Point targetPosition;     // targetPosition of robot
	vector<Task> doingTask;   // the task robot is doing
	vector<Task> todoTask;    // the queue of TodoTask
	vector<Task> doneTask;    // the queue of DoneTask
	vector<Task> updateTask;   // ��������õ���������
	vector<Point> planPath;   // the path of robot
	vector<vector<int>> workmap;  // the map robot walks
	vector<vector<WeightPoint>> weightMap;    // the map robot think the map
public:
	Robot() :Id(0), Status(0), isBack(false), waitTime(-1), bidPrice(0),
		mapRowNumber(0), mapColumnNumber(0), distance(0), fightTime(0), alreadyWait(0),
		initPosition(), nextPosition(), currentPosition(), targetPosition(){
		++robotNumber;
		cout << "Construct Robot Number " << robotNumber << endl;
		RecordLog("Construct Robot Number " + to_string(robotNumber));
	}
	bool operator == (const Robot& r) const {
		return (Id == r.Id) && (Status == r.Status) && (isBack == r.isBack) && (waitTime && r.waitTime) &&
			(bidPrice == r.bidPrice) && (mapRowNumber == r.mapRowNumber) &&
			(distance == r.distance) && (fightTime == r.fightTime) &&
			(mapColumnNumber == r.mapColumnNumber) && (initPosition == r.initPosition) &&
			(nextPosition == r.nextPosition) && (alreadyWait == r.alreadyWait) &&
			(currentPosition == r.currentPosition) && (targetPosition == r.targetPosition) &&
			(doingTask == r.doingTask) && (planPath == r.planPath) && (todoTask == r.todoTask) &&
			(doneTask == r.doneTask) && (workmap == r.workmap) && (weightMap == r.weightMap);
	}
	void Bid(Task& task);      // robot bid for task
	bool TaskOrder();    // reorder the tasks of robot
	vector<Point> ShortestPath(Point& source, Point& target);   // plan the route of robot
	vector<Point> ShortestPath(Point& source, Point& target, Point Obs);
	int ShortestDistance(Point &source, Point &target);         // ��㵽�յ�ľ���
	bool Move();         // robot move
	friend int main();
};

void 
Robot::Bid(Task& task){     // robot bid a price for the task
	float price = 0;
	Point finalPosition;
	int choice = 0;
	updateTask = doingTask;
	updateTask.push_back(task);
	if (doingTask.size() == 0){
		finalPosition = currentPosition;
		price = ShortestDistance(finalPosition, task.source) + doingTask.size() * doingTask.size();
	}
	else{
		finalPosition = doingTask.back().target;
		//̰���㷨
		if (choice == 0){
			price = ShortestDistance(finalPosition, task.source);
		}
	}
	bidTable[Id] = price;
}


bool 
Robot::TaskOrder(){          // reorder the todoTask,��ֻ����������һ���ӿڣ������Ҫ�������ڴ˽�����չ
	return true;
}


int
Robot::ShortestDistance(Point &source, Point &target){
	vector<Point> path;

	if (workmap[source.x][source.y] == 1 || workmap[target.x][target.y] == 1){
		cout << "�����յ����ϰ���" << endl;
	}

	if (source == target){
		path.push_back(source);
		return path.size();
	}
	// A* algorithm

	struct APoint{
		Point p;
		double g;
		double f;
		bool operator < (const APoint &a) const {
			return f < a.f;  // ��Сֵ����
		}
	};

	//source = Point(0, 13);
	//target = Point(17, 0);

	vector<Point> closeList;
	vector<APoint> openList;
	map<Point, Point> relation;
	APoint ap;
	ap.p = source;
	ap.g = 0;
	ap.f = 0;
	openList.push_back(ap);
	double g = 0;
	double h = 0;
	double f = 0;
	double t = 0;
	bool flag = true;
	while (openList.empty() == false && flag == true){
		ap = openList[0];
		openList.erase(openList.begin());
		Point cur = ap.p;
		closeList.push_back(cur);
		Point next;
		double gn = 0;


		for (int direct = 1; direct <= 4; ++direct){
			if (direct == 1){
				next = cur.up();
				gn = weightMap[ap.p.x][ap.p.y].up;
			}
			else if (direct == 2){
				next = cur.right();
				gn = weightMap[ap.p.x][ap.p.y].right;
			}
			else if (direct == 3){
				next = cur.down();
				gn = weightMap[ap.p.x][ap.p.y].down;
			}
			else if (direct == 4){
				next = cur.left();
				gn = weightMap[ap.p.x][ap.p.y].left;
			}
			if (next.x >= 0 && next.x < mapRowNumber && next.y >= 0 &&
				next.y < mapColumnNumber && workmap[next.x][next.y] == 0){
				//�����µ�F(N)

				int ci = -1;
				for (int i = 0; i < closeList.size(); ++i){
					if (closeList[i] == next){
						ci = i;
						break;
					}
				}

				if (ci == -1){
					int oi = -1;
					for (int i = 0; i < openList.size(); ++i){
						if (openList[i].p == next){
							oi = i;
							break;
						}
					}
					if (oi == -1){    //openList�в����ڱ����ڵ�
						g = ap.g + 1 + gn;
						h = abs(target.x - next.x) + abs(target.y - next.y);
						t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x)*(target.x - next.x) +
							(target.y - next.y)*(target.y - next.y)));
						f = g + h * t;
						APoint ap1;
						ap1.p = next;
						ap1.g = g;
						ap1.f = f;
						openList.push_back(ap1);
						relation[next] = cur;
					}
					else{           //openList�д��ڱ����ڵ�
						g = ap.g + 1 + gn;
						if (g < openList[oi].g){
							openList[oi].g = g;
							h = abs(target.x - next.x) + abs(target.y - next.y);
							t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x)*(target.x - next.x) +
								(target.y - next.y)*(target.y - next.y)));
							f = openList[oi].g + h * t;
							openList[oi].f = f;
							relation[next] = cur;
						}
					}
				}
			}
		}

		if (cur == target){
			flag = false;
			break;
		}
		sort(openList.begin(), openList.end());
	}

	vector<Point> result;

	result.push_back(target);
	Point cur = target;
	Point next = relation[cur];
	while (!(relation[cur] == source)){
		result.push_back(relation[cur]);
		cur = relation[cur];
	}

	result.push_back(source);
	
	return result.size();
}

vector<Point> 
Robot::ShortestPath(Point& source, Point& target){   // find a route from source to target
	
	vector<Point> path;

	if (workmap[source.x][source.y] == 1 || workmap[target.x][target.y] == 1){
		cout << "�����յ����ϰ���" << endl;
	}

	if (source == target){
		path.push_back(source);
		return path;
	}
	// A* algorithm

	struct APoint{
		Point p;
		double g;
		double f;
		bool operator < (const APoint &a) const {
			return f < a.f;  // ��Сֵ����
		}
	};

	vector<Point> closeList;
	vector<APoint> openList;
	map<Point, Point> relation;
	int choice = 0;
	APoint ap;
	ap.p = source;
	ap.g = 0;
	ap.f = 0;
	openList.push_back(ap);
	double g = 0;
	double h = 0;
	double f = 0;
	double t = 0;
	bool flag = true;
	while (openList.empty() == false && flag == true){
		ap = openList[0];
		openList.erase(openList.begin());
		Point cur = ap.p;
		closeList.push_back(cur);
		Point next;
		double gn = 0;


		for (int direct = 1; direct <= 4; ++direct){
			if (direct == 1){
				next = cur.up();
				gn = weightMap[ap.p.x][ap.p.y].up;
			}
			else if (direct == 2){
				next = cur.right();
				gn = weightMap[ap.p.x][ap.p.y].right;
			}
			else if (direct == 3){
				next = cur.down();
				gn = weightMap[ap.p.x][ap.p.y].down;
			}
			else if (direct == 4){
				next = cur.left();
				gn = weightMap[ap.p.x][ap.p.y].left;
			}
			if (next.x >= 0 && next.x < mapRowNumber && next.y >= 0 &&
				next.y < mapColumnNumber && workmap[next.x][next.y] == 0){
				//�����µ�F(N)

				int ci = -1;
				for (int i = 0; i < closeList.size(); ++i){
					if (closeList[i] == next){
						ci = i;
						break;
					}
				}

				if (ci == -1){
					int oi = -1;
					for (int i = 0; i < openList.size(); ++i){
						if (openList[i].p == next){
							oi = i;
							break;
						}
					}
					if (oi == -1){    //openList�в����ڱ����ڵ�
						g = ap.g + 1 + gn;
						h = abs(target.x - next.x) + abs(target.y - next.y);
						if (choice == 0){
							t = 1;
						}
						else if (choice == 1){
							t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x)*(target.x - next.x) +
								(target.y - next.y)*(target.y - next.y)));
						}
						f = g + h * t;
						APoint ap1;
						ap1.p = next;
						ap1.g = g;
						ap1.f = f;
						openList.push_back(ap1);
						relation[next] = cur;
					}
					else{           //openList�д��ڱ����ڵ�
						g = ap.g + 1 + gn;
						if (g < openList[oi].g){
							openList[oi].g = g;
							h = abs(target.x - next.x) + abs(target.y - next.y);
							if (choice == 0){
								t = 1;
							}
							else if (choice == 1){
								t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x)*(target.x - next.x) +
									(target.y - next.y)*(target.y - next.y)));
							}
							f = openList[oi].g + h * t;
							openList[oi].f = f;
							relation[next] = cur;
						}
					}
				}
			}
		}

		if ( cur == target){
			flag = false;
			break;
		}
		sort(openList.begin(),openList.end());
	}

	vector<Point> result;

	result.push_back(target);
	Point cur = target;
	Point next = relation[cur];
	while (!(relation[cur] == source)){
		result.push_back(relation[cur]);
		cur = relation[cur];
	}

	result.push_back(source);
	int low = 0, high = result.size() - 1;
	while (low < high){
		Point tmp = result[low];
		result[low] = result[high];
		result[high] = tmp;
		++low;
		--high;
	}
	return result;
}

vector<Point>
Robot::ShortestPath(Point& source, Point& target, Point Obs){   // find a route from source to target

	vector<Point> path;
	vector<vector<int>> tmpworkmap = workmap;
	tmpworkmap[Obs.x][Obs.y] = 1;
	if (tmpworkmap[source.x][source.y] == 1 || tmpworkmap[target.x][target.y] == 1){
		cout << "�����յ����ϰ���" << endl;
		return path;
	}

	if (source == target){
		path.push_back(source);
		return path;
	}
	// A* algorithm

	struct APoint{
		Point p;
		double g;
		double f;
		bool operator < (const APoint &a) const {
			return f < a.f;  // ��Сֵ����
		}
	};

	vector<Point> closeList;
	vector<APoint> openList;
	map<Point, Point> relation;
	int choice = 0;
	APoint ap;
	ap.p = source;
	ap.g = 0;
	ap.f = 0;
	openList.push_back(ap);
	double g = 0;
	double h = 0;
	double f = 0;
	double t = 0;
	bool flag = true;
	while (openList.empty() == false && flag == true){
		ap = openList[0];
		openList.erase(openList.begin());
		Point cur = ap.p;
		closeList.push_back(cur);
		Point next;
		double gn = 0;


		for (int direct = 1; direct <= 4; ++direct){
			if (direct == 1){
				next = cur.up();
				gn = weightMap[ap.p.x][ap.p.y].up;
			}
			else if (direct == 2){
				next = cur.right();
				gn = weightMap[ap.p.x][ap.p.y].right;
			}
			else if (direct == 3){
				next = cur.down();
				gn = weightMap[ap.p.x][ap.p.y].down;
			}
			else if (direct == 4){
				next = cur.left();
				gn = weightMap[ap.p.x][ap.p.y].left;
			}
			if (next.x >= 0 && next.x < mapRowNumber && next.y >= 0 &&
				next.y < mapColumnNumber && tmpworkmap[next.x][next.y] == 0){
				//�����µ�F(N)

				int ci = -1;
				for (int i = 0; i < closeList.size(); ++i){
					if (closeList[i] == next){
						ci = i;
						break;
					}
				}

				if (ci == -1){
					int oi = -1;
					for (int i = 0; i < openList.size(); ++i){
						if (openList[i].p == next){
							oi = i;
							break;
						}
					}
					if (oi == -1){    //openList�в����ڱ����ڵ�
						g = ap.g + 1 + gn;
						h = abs(target.x - next.x) + abs(target.y - next.y);
						if (choice == 0){
							t = 1;
						}
						else if (choice == 1){
							t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x)*(target.x - next.x) +
								(target.y - next.y)*(target.y - next.y)));
						}
						f = g + h * t;
						APoint ap1;
						ap1.p = next;
						ap1.g = g;
						ap1.f = f;
						openList.push_back(ap1);
						relation[next] = cur;
					}
					else{           //openList�д��ڱ����ڵ�
						g = ap.g + 1 + gn;
						if (g < openList[oi].g){
							openList[oi].g = g;
							h = abs(target.x - next.x) + abs(target.y - next.y);
							if (choice == 0){
								t = 1;
							}
							else if (choice == 1){
								t = 1.0 + 1.0 / (sqrt(1.0 * (target.x - next.x)*(target.x - next.x) +
									(target.y - next.y)*(target.y - next.y)));
							}
							f = openList[oi].g + h * t;
							openList[oi].f = f;
							relation[next] = cur;
						}
					}
				}
			}
		}

		if (cur == target){
			flag = false;
			break;
		}
		sort(openList.begin(), openList.end());
	}

	vector<Point> result;

	result.push_back(target);
	Point cur = target;
	Point next = relation[cur];
	while (!(relation[cur] == source)){
		result.push_back(relation[cur]);
		cur = relation[cur];
	}

	result.push_back(source);
	int low = 0, high = result.size() - 1;
	while (low < high){
		Point tmp = result[low];
		result[low] = result[high];
		result[high] = tmp;
		++low;
		--high;
	}
	return result;
}

bool
Robot::Move(){               //The way robot move
	// ����滮�õ�·��Ϊ�գ������¹滮·��
	if (Status == 0){   // ���������״̬Ϊ0����˵����ǰ�����Ѿ�ִ�����
		;
	}
	else if (Status == 1){
		targetPosition = doingTask[0].source;
	}
	else if (Status == 2){
		targetPosition = doingTask[0].target;
	}

	if (planPath.empty() == true){
		planPath = ShortestPath(currentPosition, targetPosition);
	}

	if (planPath.empty() == false){
		//��ӻ������ƶ��߼�
		if (waitTime == -1){
			nextPosition = planPath.front();
			mu.lock();
			auto it = Robot::robotPosition.find(nextPosition);
			if (it == Robot::robotPosition.end() || it->second == Id ){
				Robot::robotPosition.erase(currentPosition);
				Robot::robotPosition.insert(map<Point, int>::value_type(nextPosition, Id));
				currentPosition = nextPosition;
				planPath.erase(planPath.begin());
				++distance;
				alreadyWait = 0;
			}
			else{    // �����˷�����ײ
				fightTime++;
				waitTime = 3;
			}
			mu.unlock();
		}
		else if (waitTime > 0){
			nextPosition = planPath.front();
			mu.lock();
			auto it = Robot::robotPosition.find(nextPosition);
			if (it == Robot::robotPosition.end() || it->second == Id){
				Robot::robotPosition.erase(currentPosition);
				Robot::robotPosition.insert(map<Point, int>::value_type(nextPosition, Id));
				currentPosition = nextPosition;
				planPath.erase(planPath.begin());
				++distance;
				waitTime = -1;
			}
			else{
				// ���¹滮·������һ������ѡ�����¹滮·��
				++alreadyWait;
				Point p = Point(nextPosition.x - currentPosition.x, nextPosition.y - currentPosition.y);
				if (p.x == -1 && p.y == 0){
					weightMap[currentPosition.x][currentPosition.y].up += alreadyWait;
				}
				else if (p.x == 1 && p.y == 0){
					weightMap[currentPosition.x][currentPosition.y].down += alreadyWait;
				}
				else if (p.x == 0 && p.y == -1){
					weightMap[currentPosition.x][currentPosition.y].left += alreadyWait;
				}
				else if (p.x == 0 && p.y == 1){
					weightMap[currentPosition.x][currentPosition.y].right += alreadyWait;
				}
				--waitTime;
			}
			mu.unlock();
		}
		else if (waitTime == 0){
			//���¹滮·��
			planPath = ShortestPath(currentPosition, targetPosition);
			waitTime = -1;
		}
		

	}
	if (currentPosition == targetPosition){
		if (Status == 1){
			Status = 2;
		}
		else if (Status == 2){
			doingTask.erase(doingTask.begin());
			if (doingTask.empty()){  //����������������Ϊ�գ���ѡ�񷵻����
				Status = 3;
				targetPosition = initPosition;
			}
			else{    //ǰ�����
				if (doingTask.empty() == false){
					Status = 1;
				}
				else{
					Status = 3;
					targetPosition = initPosition;
				}
			}
		}
		else if (Status == 3){
			Status = 0;
		}
	}
	return true;
}

bool RecordLog(string s){
	ofstream logFile;
	logFile.open("../TestRobot/Log.txt", ofstream::app);
	if (logFile){
		logFile << s.c_str() << endl;
		logFile.close();
	}
	else{
		cout << " Failed to open the target file . The target file is Log.txt. " << endl;
		return false;
	}
	return true;
}

int Robot::robotNumber = 0;
vector<float> Robot::bidTable;
mutex Robot::mu;
map<Point, int> Robot::robotPosition;

int main(){
	//����־�ļ�
	ofstream logFile;
	logFile.open("../TestRobot/Log.txt", ofstream::out);
	if (logFile){
		logFile << "System Init ......" << endl;
		logFile.close();
	}
	else
		cout << " Failed to open the target file . The target file is Log.txt. " << endl;
	cout << "System Start ......" << endl;
	int seedOfRandom = 8141106;     // �������
	srand(seedOfRandom);
	//cout << "The seed of the random is " << seedOfRandom << endl;
	RecordLog("The seed of the random is " + to_string(seedOfRandom));
	//�������������rule
	Rule rule;
	rule.ReadMap();
	rule.ReadRobotInitPosition();
	//����rule�����������
	/*rule.GenerateTask();
	return 0;*/

	int numberTime = 5;

	while (numberTime > 0){
		rule.ReadTask();
		int curTime = 0;
		rule.setGlobalTime(curTime);
		int robotNumber = rule.getRobotNumber();
		Robot::bidTable.resize(robotNumber, INT_MAX);
		//����������
		vector<Robot> robot;
		for (int i = 0; i < robotNumber; ++i){
			Robot tmp;
			tmp.Id = i;
			tmp.initPosition = rule.getRobotInitPosition(i);
			tmp.currentPosition = rule.getRobotInitPosition(i);
			tmp.targetPosition = rule.getRobotInitPosition(i);
			tmp.workmap = rule.getMap();
			tmp.mapRowNumber = rule.getMapRowNumber();
			tmp.mapColumnNumber = rule.getMapColumnNumber();
			tmp.weightMap = rule.getWeightMap();
			tmp.Status = 0;
			robot.push_back(tmp);
			rule.setRobotCurrentPosition(i, tmp.currentPosition);
			rule.setRobotTargetPosition(i, tmp.targetPosition);
		}
		rule.RecordCurrentAndTargetPosition();
		// ���ϲ�����ϵͳ�ĳ�ʼ������

		// ����������֪��ͼ
		for (int i = 0; i < robotNumber; ++i){
			string name = "weightMap";
			name = name + to_string(i);
			ifstream logFile;
			string Position = "../TestRobot/";
			Position = Position + name + ".txt";
			logFile.open(Position, istream::in);
			if (logFile){
				for (int r = 0; r < robot[i].mapRowNumber; ++r){
					for (int c = 0; c < robot[i].mapColumnNumber; ++c){
						logFile >> robot[i].weightMap[r][c].up
							>> robot[i].weightMap[r][c].down
							>> robot[i].weightMap[r][c].left
							>> robot[i].weightMap[r][c].right;
					}
				}
				logFile.close();
			}
		}



		// ϵͳ��ʼ����
		cout << "System Start......" << endl;
		RecordLog("System Start......");

		cout << "The number of Task is " << rule.ToDoTask.size() << endl;

		double overAllBeginTime = (double)clock();
		getchar();


		int taskOrder = 0;

		int AllRobotsStatus = 0;
		for (int i = 0; i < robotNumber; ++i){
			AllRobotsStatus += robot[i].Status;
		}

		int choice = 1;

		for (int i = 0; i < robotNumber; ++i){
			Robot::robotPosition.insert(map<Point, int>::value_type(robot[i].currentPosition, i));
		}

		while (rule.ToDoTask.size() > 0 || robot[0].doingTask.size() > 0 || rule.waitToAssigned.size() > 0 || AllRobotsStatus > 0){    //ϵͳ��������
			if ((double)clock() - overAllBeginTime > 1000){   //һ������
				if (rule.ToDoTask.size() > 0 || rule.waitToAssigned.size() > 0){
					if (rule.ToDoTask.size() > 0 && rule.getGlobalTime() == rule.ToDoTask[0].publishTime){  //�����������ɵ�ʱ��
						if (choice == 0){
							if (robot[taskOrder%robotNumber].Status == 3){
								robot[taskOrder%robotNumber].planPath.clear();
							}
							if (robot[taskOrder%robotNumber].doingTask.empty() == true){
								robot[taskOrder%robotNumber].Status = 1;
							}
							robot[taskOrder%robotNumber].doingTask.push_back(rule.ToDoTask[0]);
							rule.ToDoTask.erase(rule.ToDoTask.begin());
							++taskOrder;
						}
						else{
							if (rule.waitToAssigned.size() > 0){
								rule.waitToAssigned.push_back(rule.ToDoTask[0]);
								rule.ToDoTask.erase(rule.ToDoTask.begin());
							}
							else{
								vector<thread> allthread;
								for (int i = 0; i < robotNumber; ++i){
									allthread.push_back(thread(&Robot::Bid, std::ref(robot[i]), rule.ToDoTask[0]));
								}
								for (int i = 0; i < robotNumber; ++i){
									allthread[i].join();
								}
								int targetRobot = -1, lowestPrice = INT_MAX;
								for (int i = 0; i < robotNumber; ++i){
									if (Robot::bidTable[i] < lowestPrice){
										lowestPrice = Robot::bidTable[i];
										targetRobot = i;
									}
								}
								if (lowestPrice == INT_MAX){
									rule.waitToAssigned.push_back(rule.ToDoTask[0]);
									rule.ToDoTask.erase(rule.ToDoTask.begin());
								}
								else{
									if (robot[targetRobot].Status == 3){
										robot[targetRobot].planPath.clear();
									}
									if (robot[targetRobot].doingTask.empty()){
										robot[targetRobot].Status = 1;
									}
									robot[targetRobot].doingTask = robot[targetRobot].updateTask;
									rule.ToDoTask.erase(rule.ToDoTask.begin());
									for (int i = 0; i < robotNumber; ++i){
										if (i != targetRobot){
											robot[i].updateTask.clear();
										}
									}
								}
								Robot::bidTable.resize(robotNumber, INT_MAX);
							}
						}
					}
					if (rule.waitToAssigned.size() > 0){
						vector<thread> allthread;
						for (int i = 0; i < robotNumber; ++i){
							allthread.push_back(thread(&Robot::Bid, std::ref(robot[i]), rule.waitToAssigned[0]));
						}
						for (int i = 0; i < robotNumber; ++i){
							allthread[i].join();
						}
						int targetRobot = -1, lowestPrice = INT_MAX;
						for (int i = 0; i < robotNumber; ++i){
							if (Robot::bidTable[i] < lowestPrice){
								lowestPrice = Robot::bidTable[i];
								targetRobot = i;
							}
						}
						if (lowestPrice == INT_MAX){
							;
						}
						else{
							if (robot[targetRobot].Status == 3){
								robot[targetRobot].planPath.clear();
							}
							if (robot[targetRobot].doingTask.empty()){
								robot[targetRobot].Status = 1;
							}
							robot[targetRobot].doingTask = robot[targetRobot].updateTask;
							rule.waitToAssigned.erase(rule.waitToAssigned.begin());
							for (int i = 0; i < robotNumber; ++i){
								if (i != targetRobot){
									robot[i].updateTask.clear();
								}
							}
						}
						Robot::bidTable.resize(robotNumber, INT_MAX);
					}
				}
				//getchar();



				for (int i = 0; i < robotNumber; ++i){
					cout << "robot " << i << ":";
					for (int j = 0; j < robot[i].doingTask.size(); ++j){
						cout << robot[i].doingTask[j].id << " ";
					}
					cout << endl;
					cout << robot[i].updateTask.size() << endl;
				}

				//getchar();

				// ִ������
				vector<thread> allthread;
				for (int i = 0; i < robotNumber; ++i){
					allthread.push_back(thread(&Robot::Move, std::ref(robot[i])));
				}
				for (int i = 0; i < robotNumber; ++i){
					allthread[i].join();
				}

				for (int i = 0; i < robotNumber; ++i){
					//robot[i].Move();
					rule.setRobotCurrentPosition(i, robot[i].currentPosition);
					rule.setRobotTargetPosition(i, robot[i].targetPosition);
				}
				rule.RecordCurrentAndTargetPosition();
				AllRobotsStatus = 0;
				for (int i = 0; i < robotNumber; ++i){
					AllRobotsStatus += robot[i].Status;
				}
				overAllBeginTime = (double)clock();
				rule.GlobalTime();
			}
		}
		//��ÿ�������˵���֪��ͼ���������Ա������֤
		for (int i = 0; i < robotNumber; ++i){
			string name = "weightMap";
			name = name + to_string(i);
			ofstream logFile;
			string Position = "../TestRobot/";
			Position = Position + name + ".txt";
			logFile.open(Position, ofstream::out);
			if (logFile){
				for (int r = 0; r < robot[i].mapRowNumber; ++r){
					for (int c = 0; c < robot[i].mapColumnNumber; ++c){
						logFile << robot[i].weightMap[r][c].up << " "
							<< robot[i].weightMap[r][c].down << " "
							<< robot[i].weightMap[r][c].left << " "
							<< robot[i].weightMap[r][c].right << endl;
					}
				}
				logFile.close();
			}
			else{
				cout << " Failed to open the target file . The target file is Log.txt. " << endl;
				return false;
			}
		}
		int sumDis = 0;
		for (int i = 0; i < robotNumber; ++i){
			sumDis = sumDis + robot[i].distance;
		}
		cout << "sumDis=" << sumDis << endl;
		int sumFig = 0;
		for (int i = 0; i < robotNumber; ++i){
			cout << "Roobt " << i << " FightTime:" << robot[i].fightTime << endl;
			sumFig += robot[i].fightTime;
		}
		cout << "sumFig=" << sumFig << endl;
		ofstream logFile;
		logFile.open("../TestRobot/Result.txt", ofstream::app);
		if (logFile){
			logFile << sumFig << "  ";
			logFile << rule.globalTime << "  ";
			logFile << sumDis << endl;
			logFile.close();
		}
		else{
			cout << " Failed to open the target file . The target file is Log.txt. " << endl;
			return false;
		}
		--numberTime;
	}
	
	getchar();
}