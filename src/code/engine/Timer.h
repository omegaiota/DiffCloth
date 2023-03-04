//
// Created by Yifei Li on 12/31/20.
// Email: liyifei@csail.mit.edu
//

#ifndef OMEGAENGINE_TIMER_H
#define OMEGAENGINE_TIMER_H

#include "Macros.h"

class Timer {
public:
	bool enabled;
	int numCounters;
	std::chrono::steady_clock::time_point lastTimeStep, start, end,
			debugTimeStamp;
	std::string lastLabel;
	std::vector<std::chrono::steady_clock::time_point> timeStamps;
	std::vector<long long> durationsMicroseconds;
	std::vector<std::string> labels;
	std::map<std::string, int> labelIdx;

	PerformanceTiming perf; // in [ms]

	std::chrono::steady_clock::time_point getTimeNow() {
		return std::chrono::steady_clock::now();
	}

	static std::vector<std::pair<std::string, long long>>
	addTimer(std::vector<std::pair<std::string, long long>> a,
			std::vector<std::pair<std::string, long long>> b) {
		if (a.size() != b.size()) {
			//        std::printf("ERROR: timer cannot be add up: size %zu %zu is
			//        different. return a\n", a.size(), b.size()); std::printf("labels
			//        in a:"); for (std::pair<std::string, long long>& item : a)
			//          std::printf("%s|", item.first.c_str());
			//        std::printf("\n");
			//        std::printf("labels in b:");
			//        for (std::pair<std::string, long long>& item : b)
			//          std::printf("%s|", item.first.c_str());
			//        std::printf("\n");
			return a;
		}
		for (int i = 0; i < a.size(); i++) {
			std::pair<std::string, long long> &itemA = a[i];
			std::pair<std::string, long long> itemB = b[i];
			if (itemA.first != itemB.first) {
				//          std::printf("ERROR: timer cannot be add up: label %s and %s
				//          are different. return a\n", itemA.first.c_str(),
				//          itemB.first.c_str());
			}
			itemA.second += itemB.second;
		}

		return a;
	}

	static PerformanceTiming addPerf(PerformanceTiming a, PerformanceTiming b) {
		a.solveIterativeMicroseconds += b.solveIterativeMicroseconds;
		a.solveDirectMicroseconds += b.solveDirectMicroseconds;
		a.nonSolveTimeMicroseconds += b.nonSolveTimeMicroseconds;
		return a;
	}

	long long getTimeMicroSeconds(std::chrono::steady_clock::time_point &begin,
			std::chrono::steady_clock::time_point &end) {
		long long duration =
				std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
						.count();
		return duration;
	}

	long long getTimeMilliSeconds(std::chrono::steady_clock::time_point &begin,
			std::chrono::steady_clock::time_point &end) {
		long long duration =
				std::chrono::duration_cast<std::chrono::milliseconds>(end - begin)
						.count();
		return duration;
	}

public:
	Timer() :
			enabled(false) {
		perf.solveDirectMicroseconds = 0;
		perf.solveIterativeMicroseconds = 0;
		perf.nonSolveTimeMicroseconds = 0;
		numCounters = 0;
		lastTimeStep = getTimeNow();
		debugTimeStamp = getTimeNow();
	}

	void resetAll() {
		if (!enabled)
			return;
		numCounters = 0;
		timeStamps.clear();
		labels.clear();
		durationsMicroseconds.clear();
	}

	int tic(std::string label) {
		if (!enabled)
			return -1;
		lastLabel = label;
		lastTimeStep = getTimeNow();
		return 0;
	}

	void ticStart() { start = getTimeNow(); }

	void ticEnd() {
		end = getTimeNow();
		;
	}
	int toc() {
		if (!enabled)
			return -1;
		auto end = getTimeNow();
		debugTimeStamp = end;
		auto begin = lastTimeStep;
		long long duration = getTimeMicroSeconds(begin, end);

		if (lastLabel == "solveIterative") {
			perf.solveIterativeMicroseconds += duration;
			return -1;
		}

		if (lastLabel == "solveDirect") {
			perf.solveDirectMicroseconds += duration;
			return -1;
		}

		if (labelIdx.find(lastLabel) == labelIdx.end()) {
			labelIdx[lastLabel] = labels.size();
			labels.push_back(lastLabel);
			durationsMicroseconds.push_back(duration);
		} else {
			durationsMicroseconds[labelIdx[lastLabel]] += duration;
		}

		//      long long  totalTime = 0;
		//      for (int i = 0; i < durationsMicroseconds.size(); i++) {
		//        long long  ms = durationsMicroseconds[i] ;
		//        totalTime += ms;
		//      }
		//
		//     long long startEnd = getTimeMicroSeconds(start, end);
		//      std::printf("Toc at %s iter totalAccum %lld startEnd %lld error:
		//      %.7f\n", lastLabel.c_str(), totalTime,startEnd, std::abs(totalTime -
		//      startEnd) * 1.0 / startEnd);

		return (durationsMicroseconds.size() - 1);
	}

	void report() {
		if (!enabled)
			return;
		std::printf("Timer report\n=================================\n");
		long long durationTotal = 0;
		for (long long d : durationsMicroseconds)
			durationTotal += d;
		for (int i = 0; i < durationsMicroseconds.size(); i++) {
			long long duration = durationsMicroseconds[i];
			std::printf("%s:\t%lld[µs]\t% .2f%%\n", labels[i].c_str(), duration,
					(duration * 1.0 / durationTotal) * 100.0);
		}

		std::printf("Total time: %lld\n", durationTotal);
	}

	TimerContent getReportMicroseconds() {
		std::vector<TimerEntry> times;
		TimerContent ret = {};
		int num = durationsMicroseconds.size();
		if (durationsMicroseconds.size() != labels.size()) {
			//        std::printf("ERROR: timer duration size %zu is not equal to
			//        label size %zu\n", durationsMicroseconds.size(), labels.size());
			return ret;
		}
		long long totalTime = 0;
		for (int i = 0; i < num; i++) {
			times.emplace_back(labels[i], durationsMicroseconds[i]);
			totalTime += durationsMicroseconds[i];
		}

		ret.timeMicroseconds = times;
		ret.totalMicroseconds = getTimeMicroSeconds(start, end);
		ret.solvePerfReport = getSolveTimingReport();
		return ret;
	}

	PerformanceTiming getSolveTimingReport() {
		perf.nonSolveTimeMicroseconds = getTimeMicroSeconds(start, end) -
				perf.solveDirectMicroseconds -
				perf.solveIterativeMicroseconds;
		return perf;
	};
};
#endif // OMEGAENGINE_TIMER_H
