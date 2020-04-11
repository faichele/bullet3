#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../Utils/b3Clock.h"

#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include <stdio.h>
#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "rigidbody/ConcaveScene.h"

CommonExampleInterface* example;

b3MouseMoveCallback prevMouseMoveCallback_CCScene = 0;
static void OnMouseMove(float x, float y)
{
	bool handled = false;
	handled = example->mouseMoveCallback(x, y);
	if (!handled)
	{
		if (prevMouseMoveCallback_CCScene)
			prevMouseMoveCallback_CCScene(x, y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback_CCScene = 0;
static void OnMouseDown(int button, int state, float x, float y)
{
	bool handled = false;

	handled = example->mouseButtonCallback(button, state, x, y);
	if (!handled)
	{
		if (prevMouseButtonCallback_CCScene)
			prevMouseButtonCallback_CCScene(button, state, x, y);
	}
}


int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet ConcaveScene OpenCL Example", 1024, 768, true);

	prevMouseButtonCallback_CCScene = app->m_window->getMouseButtonCallback();
	prevMouseMoveCallback_CCScene = app->m_window->getMouseMoveCallback();

	app->m_window->setMouseButtonCallback((b3MouseButtonCallback) OnMouseDown);
	app->m_window->setMouseMoveCallback((b3MouseMoveCallback) OnMouseMove);

	OpenGLGuiHelper gui(app, false);

	CommonExampleOptions options(&gui);

	example = GPUConcaveSceneCreateFunc(options);
	example->processCommandLineArgs(argc, argv);

	example->initPhysics();
	example->resetCamera();

	b3Clock clock;

	do
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		btScalar dtSec = btScalar(clock.getTimeInSeconds());
		if (dtSec > 0.1)
			dtSec = 0.1;

		example->stepSimulation(dtSec);
		clock.reset();

		example->renderScene();

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);

		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}
