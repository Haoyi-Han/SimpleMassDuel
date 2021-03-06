// Basic Lib
#include <iostream>
#include <cstring>
#include <fstream>
#include <iomanip>

// BT.CPP Lib
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>

#ifdef ZMQ_FOUND
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#endif

// MassDuel Lib
#include "mass_point_v2/mass_point_2d.hpp"
#include "mass_duel_2d_v2/hinder.h"
#include "mass_duel_2d_v2/search.h"
#include "mass_bt_nodes_v2/mass_bt_nodes_2d.hpp"

// Boost Lib
#include <boost/program_options.hpp>
#include <boost/exception/diagnostic_information.hpp>

namespace bpo = boost::program_options;

void conflicting_options(const bpo::variables_map& vm, const char* opt1, const char* opt2)
{
	if (vm.count(opt1) && !vm[opt1].defaulted() && vm.count(opt2) && !vm[opt2].defaulted())
		throw std::logic_error(std::string("Conflicting options '") + opt1 + "' and '" + opt2 + "'.");
}

int main(int argc, char** argv)
{
	std::cout << std::setprecision(4) << std::fixed;
	std::cout << "Project started ..." << std::endl;

	bpo::options_description desc("Options");
	desc.add_options()
		("help,h", "produce help message")
		("text,t", "read xml from program stored default text")
		("file,f", bpo::value<std::string>()->implicit_value("./TreeNodes.xml"), "read xml from given file path")
		("ticktime,c", bpo::value<double>()->implicit_value(0.5), "define a time period for each tick")
		("logprint,p", "display state changes in terminal")
		("logrecord,r", bpo::value<std::string>()->implicit_value("bt_trace"), "record state changes in file")
		("logmonitor,m", "use ZeroMQ to publish state changes for Groot to monitor")
		("savetrace,s", bpo::value<std::string>()->implicit_value("./trace_mass.dat"), "save the traject of mass point to file")
		("loop,l", bpo::value<int>()->implicit_value(0), "turn on loop mode to tick multi times till end");
	bpo::variables_map vm;
	try
	{
		bpo::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
		conflicting_options(vm, "text", "file");
		bpo::notify(vm);
	}
	catch (boost::exception& boost_err)
	{
		std::cerr << boost::diagnostic_information(boost_err) << std::endl;
		return false;
	}
	if (vm.empty() || vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	const bool CALLTIME = vm.count("ticktime");
	const bool SAVETRACE = vm.count("savetrace");
	if (CALLTIME) MassBTNodes2D::tick_time = vm["ticktime"].as<double>();
	if (SAVETRACE) MassBTNodes2D::tracemass_filename = vm["savetrace"].as<std::string>();

	BT::BehaviorTreeFactory factory;
	MassBTNodes2D::RegisterNodes(factory);

	BT::Tree tree;
	auto bb = BT::Blackboard::create();

	if (vm.count("text"))
	{
		std::cout << "Read from stored xml data ..." << std::endl;
		tree = factory.createTreeFromText(MassBTNodes2D::xml_text, bb);
	}
	if (vm.count("file"))
	{
		std::cout << "Read from external xml file ..." << std::endl;
		tree = factory.createTreeFromFile(vm["file"].as<std::string>(), bb);
	}
	std::cout << "Tree well created." << std::endl;
	BT::printTreeRecursively(tree.rootNode());
	std::cout << "BehaviorTree is successfully printed." << std::endl;
	
	const bool LOGPRINT = vm.count("logprint");
	const bool LOGRECORD = vm.count("logrecord");
	const bool LOGMONITOR = vm.count("logmonitor");
	const bool LOOP = vm.count("loop");

	BT::StdCoutLogger* logger_cout;
	std::string logger_filename;
	BT::FileLogger* logger_file;
	BT::MinitraceLogger* logger_minitrace;
#ifdef ZMQ_FOUND
	BT::PublisherZMQ* publisher_zmq;
#endif
	if (LOGPRINT)
	{
		logger_cout = new BT::StdCoutLogger(tree);
		std::cout << "CoutLogger Launched." << std::endl;
	}
	if (LOGRECORD)
	{
		logger_filename = vm["logrecord"].as<std::string>();
		logger_file = new BT::FileLogger(tree, (logger_filename + ".fbl").c_str());
		logger_minitrace = new BT::MinitraceLogger(tree, (logger_filename + ".json").c_str());
		std::cout << "FileLogger and MinitraceLogger Launched." << std::endl;
	}
	if (LOGMONITOR)
	{
#ifdef ZMQ_FOUND
		// This logger publish status changes using ZeroMQ. Used by Groot
		//publisher_zmq = new BT::PublisherZMQ(tree);
		std::cout << "ZMQ launched." << std::endl;
#endif
	}
	
	int loop_times(0); // 0 for infinity loop
	BT::NodeStatus status = BT::NodeStatus::RUNNING;
	if (LOOP)
	{
		loop_times = vm["loop"].as<int>();
		int count(0);
		do
		{
			status = BT::NodeStatus::RUNNING;
			// Keep on ticking until you get either a SUCCESS or FAILURE state
			while (status == BT::NodeStatus::RUNNING)
			{
				status = tree.tickRoot();
				MassBTNodes2D::SleepMS(1);   // optional sleep to avoid "busy loops"
			}
			MassBTNodes2D::SleepMS(1000);
			count++;
			std::cout << "count=" << count << std::endl;
		} while ((status != BT::NodeStatus::SUCCESS) && (count != loop_times));
	}
	else status = tree.tickRoot();

	return 0;
}