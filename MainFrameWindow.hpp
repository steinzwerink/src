#ifndef MAINFRAMEWINDOW_HPP_
#define MAINFRAMEWINDOW_HPP_

#include "Config.hpp"
#include "Widgets.hpp"

namespace Base
{
class DebugTraceFunction;
} // namespace Base

namespace View
{
class RobotWorldCanvas;
}

namespace Application
{
class LogTextCtrl;

/**
	 *
	 */
class MainFrameWindow : public Frame
{
public:
	/**
			 *
			 * @param aTitle The title which is shown in the title bar
			 */
	MainFrameWindow(const std::string &aTitle);
	/**
			 *
			 */
	virtual ~MainFrameWindow();
	/**
			 *
			 */
	Base::DebugTraceFunction &getTraceFunction() const;

protected:
	/**
			 *
			 */
	void initialise();
	/**
			 *
			 */
	MenuBar *initialiseMenuBar();
	/**
			 *
			 */
	Panel *initialiseClientPanel();
	/**
			 *
			 */
	SplitterWindow *initialiseSplitterWindow();
	/**
			 *
			 */
	Panel *initialiseLhsPanel();
	/**
			 *
			 */
	Panel *initialiseRhsPanel();
	/**
			 *
			 */
	Panel *initialiseButtonPanel();

protected:
private:
	Panel *clientPanel;
	MenuBar *menuBar;
	SplitterWindow *splitterWindow;

	Panel *lhsPanel;
	View::RobotWorldCanvas *robotWorldCanvas;

	Panel *rhsPanel;
	LogTextCtrl *logTextCtrl;
	Panel *buttonPanel;

	bool copied = false;
	Base::DebugTraceFunction *debugTraceFunction;

	void OnQuit(CommandEvent &anEvent);
	void OnWidgetDebugTraceFunction(CommandEvent &anEvent);
	void OnStdOutDebugTraceFunction(CommandEvent &anEvent);
	void OnAbout(CommandEvent &anEvent);

	void OnStartRobot(CommandEvent &anEvent);
	void OnStopRobot(CommandEvent &anEvent);
	void OnPopulate(CommandEvent &anEvent);
	void scenario_1_lhs(CommandEvent &(anEvent));
	void scenario_1_rhs(CommandEvent &(anEvent));
	void scenario_2_lhs(CommandEvent &(anEvent));
	void scenario_2_rhs(CommandEvent &(anEvent));
	void scenario_3_lhs(CommandEvent &(anEvent));
	void scenario_3_rhs(CommandEvent &(anEvent));
	void OnScenario_3(CommandEvent &(anEvent));
	void OnUnpopulate(CommandEvent &anEvent);
	void OnStartListening(CommandEvent &anEvent);
	void OnSendMessage(CommandEvent &anEvent);
	void OnCopyWorld(CommandEvent &anEvent);
	void OnStartListeningWorld( CommandEvent& anEvent);
	void OnStopListening(CommandEvent &anEvent);
};
//	class MainFrameWindow
} //namespace Application

#endif // MAINFRAMEWINDOW_HPP_
