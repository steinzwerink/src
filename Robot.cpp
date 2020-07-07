#include "Robot.hpp"
#include <sstream>
#include <ctime>
#include <chrono>
#include <algorithm>.

#include "Thread.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"
#include "Goal.hpp"
#include "WayPoint.hpp"
#include "Wall.hpp"
#include "RobotWorld.hpp"
#include "Shape2DUtils.hpp"
#include "CommunicationService.hpp"
#include "Client.hpp"
#include "Message.hpp"
#include "MainApplication.hpp"
#include "LaserDistanceSensor.hpp"

namespace Model
{
/**
	 *
	 */
Robot::Robot() : name(""),
				 size(DefaultSize),
				 position(DefaultPosition),
				 front(0, 0),
				 speed(0.0),
				 acting(false),
				 driving(false),
				 communicating(false)
{
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
}
/**
	 *
	 */
Robot::Robot(const std::string &aName) : name(aName),
										 size(DefaultSize),
										 position(DefaultPosition),
										 front(0, 0),
										 speed(0.0),
										 acting(false),
										 driving(false),
										 communicating(false)
{
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
}
/**
	 *
	 */
Robot::Robot(const std::string &aName,
			 const Point &aPosition) : name(aName),
									   size(DefaultSize),
									   position(aPosition),
									   front(0, 0),
									   speed(0.0),
									   acting(false),
									   driving(false),
									   communicating(false)
{
	std::shared_ptr<AbstractSensor> laserSensor(new LaserDistanceSensor(this));
	attachSensor(laserSensor);
}
/**
	 *
	 */
Robot::~Robot()
{
	if (driving)
	{
		stopDriving();
	}
	if (acting)
	{
		stopActing();
	}
	if (communicating)
	{
		stopCommunicating();
	}
}
/**
	 *
	 */
void Robot::setName(const std::string &aName,
					bool aNotifyObservers /*= true*/)
{
	name = aName;
	if (aNotifyObservers == true)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
Size Robot::getSize() const
{
	return size;
}
/**
	 *
	 */
bool Robot::getStop() const
{
	return stop;
}
/**
	 *
	 */
void Robot::setStop(const bool aStop)
{
	stop = aStop;
}
/**
	 *
	 */
void Robot::setSize(const Size &aSize,
					bool aNotifyObservers /*= true*/)
{
	size = aSize;
	if (aNotifyObservers == true)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
void Robot::setCollisionSize(const int &aSize,
							 bool aNotifyObservers /*= true*/)
{

	collision_size = aSize;

	if (aNotifyObservers == true)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
void Robot::setPosition(const Point &aPosition,
						bool aNotifyObservers /*= true*/)
{
	position = aPosition;
	if (aNotifyObservers == true)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
BoundedVector Robot::getFront() const
{
	return front;
}
/**
	 *
	 */
void Robot::setFront(const BoundedVector &aVector,
					 bool aNotifyObservers /*= true*/)
{
	front = aVector;
	if (aNotifyObservers == true)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
float Robot::getSpeed() const
{
	return speed;
}
/**
	 *
	 */
void Robot::setSpeed(float aNewSpeed,
					 bool aNotifyObservers /*= true*/)
{
	speed = aNewSpeed;
	if (aNotifyObservers == true)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
void Robot::startActing()
{
	acting = true;
	std::thread newRobotThread([this] { startDriving(); });
	robotThread.swap(newRobotThread);
}
/**
	 *
	 */
void Robot::stopActing()
{
	acting = false;
	driving = false;
	robotThread.join();
}
/**
	 *
	 */
void Robot::startDriving()
{
	driving = true;

	goal = RobotWorld::getRobotWorld().getGoal(this->name);

	calculateRoute(goal);

	drive(goal);
}

void Robot::stopDriving()
{
	driving = false;
}
/**
	 *
	 */
void Robot::startCommunicating()
{
	if (!communicating)
	{
		communicating = true;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port"))
		{
			localPort = Application::MainApplication::getArg("-local_port").value;
		}

		Messaging::CommunicationService::getCommunicationService().runRequestHandler(toPtr<Robot>(),
																					 static_cast<unsigned short>(std::stoi(localPort)));
	}
}
/**
	 *
	 */
void Robot::stopCommunicating()
{
	if (communicating)
	{
		communicating = false;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port"))
		{
			localPort = Application::MainApplication::getArg("-local_port").value;
		}

		Messaging::Client c1ient("localhost",
								 localPort,
								 toPtr<Robot>());
		Messaging::Message message(1, "stop");
		c1ient.dispatchMessage(message);
	}
}
/**
	 *
	 */
Region Robot::getRegion() const
{
	Point translatedPoints[] = {getFrontRight(), getFrontLeft(), getBackLeft(), getBackRight()};
	return Region(4, translatedPoints);
}
/**
	 *
	 */
bool Robot::intersects(const Region &aRegion) const
{
	Region region = getRegion();
	region.Intersect(aRegion);
	return !region.IsEmpty();
}
/**
	 *
	 */
Point Robot::getFrontLeft() const
{
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontLeft(x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontLeft(static_cast<int>((originalFrontLeft.x - position.x) * std::cos(angle) - (originalFrontLeft.y - position.y) * std::sin(angle) + position.x),
					static_cast<int>((originalFrontLeft.y - position.y) * std::cos(angle) + (originalFrontLeft.x - position.x) * std::sin(angle) + position.y));

	return frontLeft;
}
/**
	 *
	 */
Point Robot::getFrontRight() const
{
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalFrontRight(x + size.x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point frontRight(static_cast<int>((originalFrontRight.x - position.x) * std::cos(angle) - (originalFrontRight.y - position.y) * std::sin(angle) + position.x),
					 static_cast<int>((originalFrontRight.y - position.y) * std::cos(angle) + (originalFrontRight.x - position.x) * std::sin(angle) + position.y));

	return frontRight;
}
/**
	 *
	 */
Point Robot::getBackLeft() const
{
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackLeft(x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backLeft(static_cast<int>((originalBackLeft.x - position.x) * std::cos(angle) - (originalBackLeft.y - position.y) * std::sin(angle) + position.x),
				   static_cast<int>((originalBackLeft.y - position.y) * std::cos(angle) + (originalBackLeft.x - position.x) * std::sin(angle) + position.y));

	return backLeft;
}
/**
	 *
	 */
Point Robot::getBackRight() const
{
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	Point originalBackRight(x + size.x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	Point backRight(static_cast<int>((originalBackRight.x - position.x) * std::cos(angle) - (originalBackRight.y - position.y) * std::sin(angle) + position.x),
					static_cast<int>((originalBackRight.y - position.y) * std::cos(angle) + (originalBackRight.x - position.x) * std::sin(angle) + position.y));

	return backRight;
}
/**
	 *
	 */
void Robot::handleNotification()
{
	//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

	static int update = 0;
	if ((++update % 200) == 0)
	{
		notifyObservers();
	}
}
/**
	 *
	 */
void Robot::handleRequest(Messaging::Message &aMessage)
{
	switch (aMessage.getMessageType())
	{
	case EchoRequest:
	{
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string(": EchoRequest"));
		aMessage.setMessageType(EchoResponse);
		aMessage.setBody(": case 1 " + aMessage.asString());
		break;
	}
	case GetRobotRequest:
	{
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string(": Robotrequest"));
		aMessage.setMessageType(GetRobotResponse);
		aMessage.setBody(": case 2 " + aMessage.asString());
		break;
	}

	default:
	{
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string(": default"));

		aMessage.setBody(" default  Goodbye cruel world!");
		break;
	}
	}
}
/**
 *
 */
void Robot::handleResponse(const Messaging::Message &aMessage)
{
	switch (aMessage.getMessageType())
	{
	case EchoResponse:
	{
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string(": case EchoResponse: not implemented, ") + aMessage.asString());

		break;
	}
	case GetRobotResponse:
	{
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string("je hebt de positie van de robot binnen ") + aMessage.asString());

		break;
	}
	default:
	{
		std::cout << aMessage.getMessageType() << std::endl;
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string(": default not implemented, ") + aMessage.asString());
		break;
	}
	}
}
/**
	 *
	 */
std::string Robot::asString() const
{
	std::ostringstream os;

	os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

	return os.str();
}
/**
	 *
	 */
std::string Robot::asDebugString() const
{
	std::ostringstream os;

	os << "Robot:\n";
	os << AbstractAgent::asDebugString();
	os << "Robot " << name << " at (" << position.x << "," << position.y << ")\n";

	return os.str();
}
/**
	 *
	 */
std::string Robot::asCopyString() const
{
	std::ostringstream os;

	os << name << " " << position.x << " " << position.y << " " << front.x
	   << " " << front.y;

	return os.str();
}

void Robot::setRestarted(bool iRestarted)
{
	restarted = iRestarted;
}

/**
 *
 */
/**
 *
 */

void Robot::drive(WayPointPtr aGoal)
{

	try
	{
		for (std::shared_ptr<AbstractSensor> sensor : sensors)
		{
			//sensor->setOn();
		}

		if (speed == 0.0)
		{
			speed = 10.0;
		}

		std::vector<Model::RobotPtr> robots = RobotWorld::getRobotWorld().getRobots();

		sort(robots.begin(), robots.end(), compareRobots());
		auto myRobot = robots[0];
		auto otherRobot = robots[1];

		unsigned pathPoint = 0;
		while (position.x > 0 && position.x < 500 && position.y > 0 && position.y < 500 && pathPoint < path.size())
		{
			
			const PathAlgorithm::Vertex &vertex = path[pathPoint += static_cast<int>(speed)];
			front = BoundedVector(vertex.asPoint(), position);
			position.x = vertex.x;
			position.y = vertex.y;
			if (arrived(aGoal))
			{
				Application::Logger::log(__PRETTY_FUNCTION__ + std::string(": arrived"));
				sendCopyRobots();
				restartOtherRobot();
				
				notifyObservers();
				driving = false;
				break;
			}
			if (this->restarted == true)
			{
				std::cout<<this->name << " Restarted " <<std::endl;
				calculateRoute(goal);
				//	recalculatedNewPath = true;
				driving = true;
				drive(goal);
			}

			if (collision_robot(robots))
			{
				Application::Logger::log(__PRETTY_FUNCTION__ + std::string(":collision with robot"));
				sendCopyRobots();
				notifyObservers();

			    stopOtherRobot();

				if (this->getStop() == true)
				{

					this->stopDriving();
					std::this_thread::sleep_for(std::chrono::milliseconds(5000));
				}

				if (this->getStop() == false)
				{
					calculateRoute(goal);
					//	recalculatedNewPath = true;
					driving = true;
					drive(goal);
				}
				break;
			}
			if (collision_walls())
			{
				Application::Logger::log(__PRETTY_FUNCTION__ + std::string(":collision with wall"));
				sendCopyRobots();
				notifyObservers();
				break;
			}

			sendCopyRobots();
			notifyObservers();

			std::this_thread::sleep_for(std::chrono::milliseconds(100));

			// this should be the last thing in the loop
			if (driving == false)
			{
				break;
			}
		} // while

		for (std::shared_ptr<AbstractSensor> sensor : sensors)
		{
			//sensor->setOff();
		}
	}
	catch (std::exception &e)
	{
		std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
	}
	catch (...)
	{
		std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
	}
}

void Robot::restartOtherRobot()
{
	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12345";

	if (Application::MainApplication::isArgGiven("-remote_ip"))
	{
		remoteIpAdres = Application::MainApplication::getArg("-remote_ip").value;
	}
	if (Application::MainApplication::isArgGiven("-remote_port"))
	{
		remotePort = Application::MainApplication::getArg("-remote_port").value;
	}

	Model::RobotWorldPtr worldptr =
		Model::RobotWorld::getRobotWorld().getRobotWorldPtr();
	if (worldptr)
	{

		Messaging::Client client(remoteIpAdres, remotePort, worldptr);
		Messaging::Message message(
			Model::RobotWorld::MessageType::RestartRobotRequest,
			"Test");

		client.dispatchMessage(message);
	}
}

void Robot::stopOtherRobot()
{
	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12345";

	if (Application::MainApplication::isArgGiven("-remote_ip"))
	{
		remoteIpAdres = Application::MainApplication::getArg("-remote_ip").value;
	}
	if (Application::MainApplication::isArgGiven("-remote_port"))
	{
		remotePort = Application::MainApplication::getArg("-remote_port").value;
	}

	Model::RobotWorldPtr worldptr =
		Model::RobotWorld::getRobotWorld().getRobotWorldPtr();
	if (worldptr)
	{

		Messaging::Client client(remoteIpAdres, remotePort, worldptr);
		Messaging::Message message(
			Model::RobotWorld::MessageType::StopRobotRequest,
			"Test");

		client.dispatchMessage(message);
	}
}

void Robot::sendCopyRobots()
{
	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12345";

	if (Application::MainApplication::isArgGiven("-remote_ip"))
	{
		remoteIpAdres = Application::MainApplication::getArg("-remote_ip").value;
	}
	if (Application::MainApplication::isArgGiven("-remote_port"))
	{
		remotePort = Application::MainApplication::getArg("-remote_port").value;
	}
	// We will request an echo message. The response will be "Hello World", if all goes OK,
	// "Goodbye cruel world!" if something went wrong.

	Model::RobotWorldPtr worldptr =
		Model::RobotWorld::getRobotWorld().getRobotWorldPtr();
	if (worldptr)
	{

		Model::RobotPtr robot = (Model::RobotWorld::getRobotWorld().getRobots()[0]);

		std::string newMessage = "";

		newMessage += robot->asCopyString();
		newMessage += "\n";

		Messaging::Client client(remoteIpAdres, remotePort, worldptr);
		Messaging::Message message(
			Model::RobotWorld::MessageType::CopyRobotsRequest,
			Model::RobotWorld::getRobotWorld().asCopyString());
		message.setBody(newMessage);
		client.dispatchMessage(message);
	}
}

/**
	 *
	 */
void Robot::calculateRoute(GoalPtr aGoal)
{
	path.clear();
	if (aGoal)
	{
		// Turn off logging if not debugging AStar
		Application::Logger::setDisable();

		front = BoundedVector(aGoal->getPosition(), position);
		handleNotificationsFor(astar);
		path = astar.search(position, aGoal->getPosition(), size);
		stopHandlingNotificationsFor(astar);

		Application::Logger::setDisable(false);
	}
}
/**
	 *
	 */
bool Robot::arrived(WayPointPtr aGoal)
{
	if (aGoal)
	{
		int distanceX = abs(position.x - aGoal->getPosition().x);
		int distanceY = abs(position.y - aGoal->getPosition().y);

		if (distanceX < 10 && distanceY < 10)
			return true;
	}
	return false;
}
/**
	 *
	 */
bool Robot::collision_walls()
{
	Point frontLeft = getFrontLeft();
	Point frontRight = getFrontRight();
	Point backLeft = getBackLeft();
	Point backRight = getBackRight();

	const std::vector<WallPtr> &walls = RobotWorld::getRobotWorld().getWalls();
	for (WallPtr wall : walls)
	{
		if (Utils::Shape2DUtils::intersect(frontLeft, frontRight, wall->getPoint1(), wall->getPoint2()) ||
			Utils::Shape2DUtils::intersect(frontLeft, backLeft, wall->getPoint1(), wall->getPoint2()) ||
			Utils::Shape2DUtils::intersect(frontRight, backRight, wall->getPoint1(), wall->getPoint2()))
		{
			return true;
		}
	}
	return false;
}
bool Robot::collision_robot(std::vector<Model::RobotPtr> allRobots)
{

	Model::RobotPtr myRobot = allRobots[1];
	Model::RobotPtr otherRobot = allRobots[0];

	int distanceX = abs(myRobot->getPosition().x - otherRobot->getPosition().x);
	int distanceY = abs(myRobot->getPosition().y - otherRobot->getPosition().y);

	if (distanceX < collision_size && distanceY < collision_size)
	{
		return true;
	}
	return false;
}

Model::RobotPtr Robot::getOtherRobot(std::vector<Model::RobotPtr> allRobots, Model::RobotPtr myRobot)
{
	Model::RobotPtr otherRobot;

	for (const auto &robot : allRobots)
	{
		if (robot != myRobot)
		{
			otherRobot = robot;
		}
	}
	return otherRobot;
}

std::thread &Robot::getRobotThread()
{
	return robotThread;
}
} // namespace Model
