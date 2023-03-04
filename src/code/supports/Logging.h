//
// Created by Yifei Li on 5/23/21.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_LOGGING_H
#define OMEGAENGINE_LOGGING_H

#include <iostream>

// https://www.lihaoyi.com/post/BuildyourownCommandLinewithANSIescapecodes.html
class Logging {
	static std::string RED_START, BLACK_START, WHITE_START, YELLOW_START,
			MAGENTA_START, CYAN_START, GREEN_START, BLUE_START;
	static std::string RESET;

public:
	enum LogColor {
		RED,
		WHITE,
		BLACK,
		GREEN,
		YELLOW,
		BLUE,
		CYAN,
		MAGENTA,
		TOTAL_OPTIONS
	};

	static std::string logFatal(std::string content) {
		return logColor(content, LogColor::RED);
	}

	static std::string logWarning(std::string content) {
		return logColor(content, LogColor::YELLOW);
	}

	static std::string logOk(std::string content) {
		return logColor(content, LogColor::GREEN);
	}

	static std::string logNormal(std::string content) {
		return logColor(content, LogColor::WHITE);
	}

	static std::string logMagenta(std::string content) {
		return logColor(content, LogColor::MAGENTA);
	}

	static std::string logColor(std::string content, LogColor color) {
		std::string start = BLACK_START;
		switch (color) {
			case LogColor::RED:
				start = RED_START;
				break;
			case LogColor::YELLOW:
				start = YELLOW_START;
				break;
			case LogColor::GREEN:
				start = GREEN_START;
				break;
			case LogColor::BLACK:
				start = BLACK_START;
				break;
			case LogColor::BLUE:
				start = BLUE_START;
				break;
			case LogColor::MAGENTA:
				start = MAGENTA_START;
				break;
			case LogColor::CYAN:
				start = CYAN_START;
				break;
			case LogColor::WHITE:
				start = WHITE_START;
				break;
			case LogColor::TOTAL_OPTIONS:
				start = WHITE_START;
				break;
		}

		std::string formatted = start + content + RESET;
		std::printf("%s", formatted.c_str());
		return formatted;
	}
};

#endif // OMEGAENGINE_LOGGING_H
