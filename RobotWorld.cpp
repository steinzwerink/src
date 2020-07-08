#include "RobotWorld.hpp"
#include "Logger.hpp"
#include "Robot.hpp"
#include "WayPoint.hpp"
#include "Goal.hpp"
#include "Wall.hpp"
#include <algorithm>

#include "CommunicationService.hpp"
#include "Client.hpp"
#include "Message.hpp"
#include "MainApplication.hpp"

#include <boost/algorithm/string.hpp>
namespace Model
{
	/**
	 *
	 */
	/* static */ RobotWorld &RobotWorld::RobotWorld::getRobotWorld()
	{
		static RobotWorld robotWorld;
		return robotWorld;
	}
	/**
	 *
	 */
	RobotPtr RobotWorld::newRobot(const std::string &aName /*= "New Robot"*/,
								  const Point &aPosition /*= Point(-1,-1)*/,
								  bool aNotifyObservers /*= true*/)
	{
		RobotPtr robot(new Model::Robot(aName, aPosition));
		robots.push_back(robot);
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
		return robot;
	}
	/**
	 *
	 */
	WayPointPtr RobotWorld::newWayPoint(const std::string &aName /*= "new WayPoint"*/,
										const Point &aPosition /*= Point(-1,-1)*/,
										bool aNotifyObservers /*= true*/)
	{
		WayPointPtr wayPoint(new Model::WayPoint(aName, aPosition));
		wayPoints.push_back(wayPoint);
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
		return wayPoint;
	}
	/**
	 *
	 */
	GoalPtr RobotWorld::newGoal(const std::string &aName /*= "New Goal"*/,
								const Point &aPosition /*= Point(-1,-1)*/,
								bool aNotifyObservers /*= true*/)
	{
		GoalPtr goal(new Model::Goal(aName, aPosition));
		goals.push_back(goal);
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
		return goal;
	}
	/**
	 *
	 */
	WallPtr RobotWorld::newWall(const Point &aPoint1,
								const Point &aPoint2,
								bool aNotifyObservers /*= true*/)
	{
		WallPtr wall(new Model::Wall(aPoint1, aPoint2));
		walls.push_back(wall);
		if (aNotifyObservers == true)
		{
			notifyObservers();
		}
		return wall;
	}
	/**
	 *
	 */
	void RobotWorld::deleteRobot(RobotPtr aRobot,
								 bool aNotifyObservers /*= true*/)
	{
		auto i = std::find_if(robots.begin(), robots.end(), [aRobot](RobotPtr r) {
			return aRobot->getName() == r->getName();
		});
		if (i != robots.end())
		{
			robots.erase(i);
			if (aNotifyObservers == true)
			{
				notifyObservers();
			}
		}
	}
	/**
	 *
	 */
	void RobotWorld::deleteWayPoint(WayPointPtr aWayPoint,
									bool aNotifyObservers /*= true*/)
	{
		auto i = std::find_if(wayPoints.begin(), wayPoints.end(), [aWayPoint](WayPointPtr w) {
			return aWayPoint->getName() == w->getName();
		});
		if (i != wayPoints.end())
		{
			wayPoints.erase(i);
			if (aNotifyObservers == true)
			{
				notifyObservers();
			}
		}
	}
	/**
	 *
	 */
	void RobotWorld::deleteGoal(GoalPtr aGoal,
								bool aNotifyObservers /*= true*/)
	{
		auto i = std::find_if(goals.begin(), goals.end(), [aGoal](GoalPtr g) {
			return aGoal->getName() == g->getName();
		});
		if (i != goals.end())
		{
			goals.erase(i);

			if (aNotifyObservers == true)
			{
				notifyObservers();
			}
		}
	}
	/**
	 *
	 */
	void RobotWorld::deleteWall(WallPtr aWall,
								bool aNotifyObservers /*= true*/)
	{
		auto i = std::find_if(walls.begin(), walls.end(), [aWall](WallPtr w) {
			return aWall->getPoint1() == w->getPoint1() &&
				   aWall->getPoint2() == w->getPoint2();
		});
		if (i != walls.end())
		{
			walls.erase(i);

			if (aNotifyObservers == true)
			{
				notifyObservers();
			}
		}
	}
	/**
	 *
	 */
	RobotPtr RobotWorld::getRobot(const std::string &aName) const
	{
		for (RobotPtr robot : robots)
		{
			if (robot->getName() == aName)
			{
				return robot;
			}
		}
		return nullptr;
	}
	/**
	 *
	 */
	RobotPtr RobotWorld::getRobot(const Base::ObjectId &anObjectId) const
	{
		for (RobotPtr robot : robots)
		{
			if (robot->getObjectId() == anObjectId)
			{
				return robot;
			}
		}
		return nullptr;
	}
	/**
	 *
	 */
	WayPointPtr RobotWorld::getWayPoint(const std::string &aName) const
	{
		for (WayPointPtr wayPoint : wayPoints)
		{
			if (wayPoint->getName() == aName)
			{
				return wayPoint;
			}
		}
		return nullptr;
	}
	/**
	 *
	 */
	WayPointPtr RobotWorld::getWayPoint(const Base::ObjectId &anObjectId) const
	{
		for (WayPointPtr wayPoint : wayPoints)
		{
			if (wayPoint->getObjectId() == anObjectId)
			{
				return wayPoint;
			}
		}
		return nullptr;
	}
	/**
	 *
	 */
	GoalPtr RobotWorld::getGoal(const std::string &aName) const
	{
		for (GoalPtr goal : goals)
		{
			if (goal->getName() == aName)
			{
				return goal;
			}
		}
		return nullptr;
	}
	/**
	 *
	 */
	GoalPtr RobotWorld::getGoal(const Base::ObjectId &anObjectId) const
	{
		for (GoalPtr goal : goals)
		{
			if (goal->getObjectId() == anObjectId)
			{
				return goal;
			}
		}
		return nullptr;
	}
	/**
	 *
	 */
	WallPtr RobotWorld::getWall(const Base::ObjectId &anObjectId) const
	{
		for (WallPtr wall : walls)
		{
			if (wall->getObjectId() == anObjectId)
			{
				return wall;
			}
		}
		return nullptr;
	}

	/**
	 *
	 */
	const std::vector<RobotPtr> &RobotWorld::getRobots() const
	{
		return robots;
	}
	/**
	 *
	 */
	const std::vector<WayPointPtr> &RobotWorld::getWayPoints() const
	{
		return wayPoints;
	}
	/**
	 *
	 */
	const std::vector<GoalPtr> &RobotWorld::getGoals() const
	{
		return goals;
	}
	/**
	 *
	 */
	const std::vector<WallPtr> &RobotWorld::getWalls() const
	{
		return walls;
	}
	/**
	 *
	 */
	void RobotWorld::populate(int aNumberOfWalls /*= 2*/)
	{
		RobotWorld::getRobotWorld().newRobot("Robot", Point(163, 111), false);

		RobotWorld::getRobotWorld().newWall(Point(7, 234), Point(419, 234), false);
		RobotWorld::getRobotWorld().newGoal("Goal", Point(320, 285), false);

		notifyObservers();
	}
	/**
	 *
	 */

	void RobotWorld::populateScenario_1_lhs(int aNumberOfWalls)
	{
		RobotWorld::getRobotWorld().newRobot("Stein", Point(62, 255), false);

		static Point coordinates[] = {Point(0, 190), Point(186, 190), Point(186, 190), Point(186, 0),
									  Point(335, 0), Point(335, 193), Point(335, 193), Point(500, 193),
									   Point(500, 0), Point(500, 500), Point(0, 500), Point(500, 500),
									  Point(0, 0), Point(0, 600), Point(0, 0), Point(600, 0), Point(0, 335),
									   Point(184, 335), Point(335, 335), Point(500, 335), Point(335, 335),
									    Point(335, 500), Point(184, 335), Point(184, 500)};

		for (int i = 0; i < 2 * aNumberOfWalls; i += 2)
		{
			RobotWorld::getRobotWorld().newWall(coordinates[i], coordinates[i + 1], false);
		}
		RobotWorld::getRobotWorld().newGoal("Stein", Point(429, 258), false);

		notifyObservers();
	}

	void RobotWorld::populateScenario_1_rhs(int aNumberOfWalls)
	{
		RobotWorld::getRobotWorld().newRobot("Thomas", Point(262, 440), false);

		RobotWorld::getRobotWorld().newGoal("Thomas", Point(258, 37), false);

		notifyObservers();
	}
	/**
	 *
	 */
	void RobotWorld::populateScenario_2_lhs(int aNumberOfWalls)
	{
		RobotWorld::getRobotWorld().newRobot("Stein", Point(41, 155), false);

		static Point coordinates[] = {Point(0, 200), Point(300, 200), Point(200, 300), Point(600, 300),
									  Point(0, 0), Point(0, 600), Point(0, 0), Point(600, 0), Point(500, 0), Point(500, 500), Point(0, 500), Point(500, 500)};

		for (int i = 0; i < 2 * aNumberOfWalls; i += 2)
		{
			RobotWorld::getRobotWorld().newWall(coordinates[i], coordinates[i + 1], false);
		}
		RobotWorld::getRobotWorld().newGoal("Stein", Point(46, 232), false);

		notifyObservers();
	}
	void RobotWorld::populateScenario_2_rhs(int aNumberOfWalls)
	{
		RobotWorld::getRobotWorld().newRobot("Thomas", Point(460, 354), false);

		RobotWorld::getRobotWorld().newGoal("Thomas", Point(462, 260), false);

		notifyObservers();
	}

	void RobotWorld::populateScenario_3_lhs(int aNumberOfWalls)
	{
		RobotWorld::getRobotWorld().newRobot("Stein", Point(41, 155), false);

		static Point coordinates[] = {Point(0, 200), Point(300, 200), Point(200, 300), Point(600, 300),
									  Point(0, 0), Point(0, 600), Point(0, 0), Point(600, 0), Point(500, 0), Point(500, 500), Point(0, 500), Point(500, 500)};

		for (int i = 0; i < 2 * aNumberOfWalls; i += 2)
		{
			RobotWorld::getRobotWorld().newWall(coordinates[i], coordinates[i + 1], false);
		}
		RobotWorld::getRobotWorld().newGoal("Stein", Point(46, 232), false);

		notifyObservers();
	}
	void RobotWorld::populateScenario_3_rhs(int aNumberOfWalls)
	{
		RobotWorld::getRobotWorld().newRobot("Thomas", Point(460, 354), false);

		RobotWorld::getRobotWorld().newGoal("Thomas", Point(462, 260), false);

		notifyObservers();
	}

	void RobotWorld::unpopulate(bool aNotifyObservers /*= true*/)
	{
		robots.clear();
		wayPoints.clear();
		goals.clear();
		walls.clear();

		if (aNotifyObservers)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */
	void RobotWorld::unpopulate(const std::vector<Base::ObjectId> &aKeepObjects,
								bool aNotifyObservers /*= true*/)
	{
		if (robots.size() > 0)
		{
			robots.erase(std::remove_if(robots.begin(),
										robots.end(),
										[&aKeepObjects](RobotPtr aRobot) {
											return std::find(aKeepObjects.begin(),
															 aKeepObjects.end(),
															 aRobot->getObjectId()) == aKeepObjects.end();
										}),
						 robots.end());
		}
		if (wayPoints.size() > 0)
		{
			wayPoints.erase(std::remove_if(wayPoints.begin(),
										   wayPoints.end(),
										   [&aKeepObjects](WayPointPtr aWayPoint) {
											   return std::find(aKeepObjects.begin(),
																aKeepObjects.end(),
																aWayPoint->getObjectId()) == aKeepObjects.end();
										   }),
							wayPoints.end());
		}
		if (goals.size() > 0)
		{
			goals.erase(std::remove_if(goals.begin(),
									   goals.end(),
									   [&aKeepObjects](GoalPtr aGoal) {
										   return std::find(aKeepObjects.begin(),
															aKeepObjects.end(),
															aGoal->getObjectId()) == aKeepObjects.end();
									   }),
						goals.end());
		}
		if (walls.size() > 0)
		{
			walls.erase(std::remove_if(walls.begin(),
									   walls.end(),
									   [&aKeepObjects](WallPtr aWall) {
										   return std::find(aKeepObjects.begin(),
															aKeepObjects.end(),
															aWall->getObjectId()) == aKeepObjects.end();
									   }),
						walls.end());
		}

		if (aNotifyObservers)
		{
			notifyObservers();
		}
	}
	/**
	 *
	 */

	void Model::RobotWorld::handleRequest(Messaging::Message &aMessage)
	{
		switch (aMessage.getMessageType())
		{
		case CopyWorldRequest:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": CopyWorlds ") + aMessage.getBody());
			//Read string and create objects.
			std::string myString = aMessage.getBody();
			aMessage.setMessageType(MessageType::CopyWorldResponse);
			aMessage.setBody(this->asCopyString());
			setWorld(myString);
			break;
		}
		case CopyRobotsRequest:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": Robotrequest"));

			std::string allRobots = aMessage.getBody();
			std::string aRobotName;
			unsigned long x;
			unsigned long y;
			unsigned long lx;
			unsigned long ly;

			std::stringstream ss(allRobots);
			std::string to;

			while (std::getline(ss, to, '\n'))
			{
				std::stringstream incomingString(to);
				incomingString >> aRobotName >> x >> y >> lx >> ly;

				Model::RobotPtr robot =
					(Model::RobotWorld::getRobotWorld().getRobot(aRobotName));
				if (robot)
				{
					robot->setPosition(Point(x, y), true);
					robot->setFront(BoundedVector(lx, ly), true);
				}
			}

			break;
		}
		case StartRobotRequest:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": StartRequest"));
			Model::RobotPtr robot =
				(Model::RobotWorld::getRobotWorld().getRobots())[0];
			if (robot && !robot->isActing())
			{
				robot->startActing();
			}
			else
			{
				robot->getRobotThread().join();
				std::thread newRobotThread([this, robot] { robot->startDriving(); });
				robot->getRobotThread().swap(newRobotThread);
			}
			aMessage.setMessageType(StartRobotResponse);
			// if (robot->isDriving())
			// {
			// 	aMessage.setBody("Started remote robot.");
			// }
			// else
			// {
			// 	aMessage.setBody("Unable to start remote robot.");
			// }
			break;
		}
		case StopRobotRequest:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": StopRequest"));

			std::vector<Model::RobotPtr> robots = RobotWorld::getRobotWorld().getRobots();

			sort(robots.begin(), robots.end(), Robot::compareRobots());
			auto myRobot = robots[0];
			auto otherRobot = robots[1];

			otherRobot->setStop(true);

			break;
		}

		case RestartRobotRequest:
		{
			std::cout << "Hier ben ik in de request"<< std::endl;

			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": StopRequest"));

			std::vector<Model::RobotPtr> robots = RobotWorld::getRobotWorld().getRobots();

			sort(robots.begin(), robots.end(), Robot::compareRobots());
			auto myRobot = robots[0];
			auto otherRobot = robots[1];
			otherRobot->setStop(false);
			otherRobot->setRestarted(true);
			otherRobot->restartTest();

			break;
		}

		default:
			break;
		}
	}
	/**
 *
 */
	void Model::RobotWorld::handleResponse(const Messaging::Message &aMessage)
	{
		switch (aMessage.getMessageType())
		{

		case CopyWorldResponse:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": CopyWorlds ") + aMessage.getBody());
			//Read string and create objects.
			std::string myString = aMessage.getBody();
			setWorld(myString);
			break;
		}
		case StartRobotResponse:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string("Started other robot ") + aMessage.asString());
			break;
		}
		case StopRobotResponse:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string("Stopped other robot ") + aMessage.asString());
			std::string stopStatus = aMessage.getBody();
			break;
		}

		default:
		{
			Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": Unknown other type") + aMessage.getBody());
			break;
		}
		}
	}

	void Model::RobotWorld::setWorld(std::string &messageBody)
	{
		Application::Logger::log(
			__PRETTY_FUNCTION__ + std::string(": World") + messageBody);
		std::vector<std::string> lines;
		boost::split(lines, messageBody, boost::is_any_of("\n"));

		for (std::string line : lines)
		{
			if (!line.empty())
			{

				unsigned long incomingX;
				unsigned long incomingY;
				unsigned long incomingSecondX;
				unsigned long IncomingSecondY;

				std::stringstream incomingString;
				std::string incomingName;

				switch (std::stoi(&line.at(0)))
				{
				case Goal:
					line.erase(line.begin());
					incomingString << line;
					incomingString >> incomingName >> incomingX >> incomingY;
					newGoal(incomingName, Point(incomingX, incomingY), false);
					break;

				case Robot:
					line.erase(line.begin());
					incomingString << line;
					incomingString >> incomingName >> incomingX >> incomingY;
					newRobot((incomingName), Point(incomingX, incomingY));
					break;

				case Wall:
					line.erase(line.begin());
					incomingString << line;
					incomingString >> incomingX >> incomingY >> incomingSecondX >> IncomingSecondY;
					newWall(Point(incomingX, incomingY), Point(incomingSecondX, IncomingSecondY),
							false);
					break;

				case WayPoint:
					line.erase(line.begin());
					incomingString << line;
					incomingString >> incomingName >> incomingX >> incomingY;
					newWayPoint(incomingName, Point(incomingX, incomingY), false);
					break;

				default:
					Application::Logger::log("object unknown");
					Application::Logger::log(line);
					break;
				}
			}
		}
		Application::Logger::log("Copied world");
		notifyObservers();
	}
	std::string RobotWorld::asString() const
	{
		return ModelObject::asString();
	}
	/**
	 *
	 */
	std::string RobotWorld::asCopyString() const
	{
		std::ostringstream os;

		for (RobotPtr ptr : robots)
		{
			os << Robot << " " << ptr->asCopyString() << '\n';
		}
		for (WayPointPtr ptr : wayPoints)
		{
			os << WayPoint << " " << ptr->asCopyString() << '\n';
		}
		for (GoalPtr ptr : goals)
		{
			os << Goal << " " << ptr->asCopyString() << '\n';
		}
		for (WallPtr ptr : walls)
		{
			os << Wall << " " << ptr->asCopyString() << '\n';
		}

		return os.str();
	}
	/**
 *
 */

	void Model::RobotWorld::startCommunicating()
	{
		if (!communicating)
		{
			communicating = true;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven("-local_port"))
			{
				localPort =
					Application::MainApplication::getArg("-local_port").value;
			}

			Messaging::CommunicationService::getCommunicationService().runRequestHandler(
				toPtr<RobotWorld>(), std::stoi(localPort));
			Application::Logger::log("Started listening for world");
		}
	}

	void Model::RobotWorld::stopCommunicating()
	{
		if (communicating)
		{
			communicating = false;

			std::string localPort = "12345";
			if (Application::MainApplication::isArgGiven("-local_port"))
			{
				localPort =
					Application::MainApplication::getArg("-local_port").value;
			}

			Messaging::Client c1ient("localhost", localPort, toPtr<RobotWorld>());
			Messaging::Message message(1, "stop");
			c1ient.dispatchMessage(message);
			Application::Logger::log("Stopped listening for world");
		}
	}

	std::string RobotWorld::asDebugString() const
	{
		std::ostringstream os;

		os << asString() << '\n';

		for (RobotPtr ptr : robots)
		{
			os << ptr->asDebugString() << '\n';
		}
		for (WayPointPtr ptr : wayPoints)
		{
			os << ptr->asDebugString() << '\n';
		}
		for (GoalPtr ptr : goals)
		{
			os << ptr->asDebugString() << '\n';
		}
		for (WallPtr ptr : walls)
		{
			os << ptr->asDebugString() << '\n';
		}

		return os.str();
	}
	/**
	 *
	 */
	RobotWorld::RobotWorld()
		: communicating(false), robotWorldPtr(this)
	{
	}
	/**
	 *
	 */
	RobotWorld::~RobotWorld()
	{
		// No notification while I am in the destruction mode!
		disableNotification();
		unpopulate();
	}

} // namespace Model
