//STD
#include <stdexcept>
#include <iostream>

//LIBS
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

int main(int argc, char** argv)
{	
	doctest::Context context;
	context.applyCommandLine(argc, argv);
	context.setOption("no-breaks", true);
	context.setOption("--version", true);
	context.setOption("--count", true);
	context.setOption("--list-test-cases", true);
	context.setOption("--list-test-suites", true);
	context.setOption("--success", false);
	context.setOption("--exit", true);
	int result = context.run();
	if (context.shouldExit())
		return result;

	return result;
}