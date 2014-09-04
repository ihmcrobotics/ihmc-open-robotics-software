#ifndef __ICP_CHAIN_CREATION_H
#define __ICP_CHAIN_CREATION_H

#include <limits>
#include <boost/format.hpp>
#include "aliases.h"

template<typename R>
typename R::TargetType* initModuleFromRegistrar(const std::string& root, const R& registrar)
{
	typedef typename R::ClassDescriptor ClassDescriptor;
	typedef PointMatcherSupport::Parametrizable::ParameterDoc ParameterDoc;
	typedef PointMatcherSupport::Parametrizable::Parameters Parameters;
	typedef PointMatcherSupport::Parametrizable::ParametersDoc::const_iterator cIter;
	typedef PointMatcherSupport::Parametrizable::ParametersDoc ParametersDoc;
	
	const std::string name(getParam<std::string>(root+"/name"));
	
	const ClassDescriptor* desc(registrar.getDescriptor(name));
	Parameters params;
	PointMatcherSupport::Parametrizable::ParametersDoc paramVec = desc->availableParameters();

	auto paramsDoc = desc->availableParameters();

	// NOTE: the line above is doing segFault with doc(*it)...
	//for (auto it = desc->availableParameters().cbegin(); it != desc->availableParameters().cend(); ++it)
	for (auto it = paramsDoc.cbegin(); it != paramsDoc.cend(); ++it)
	{
		// TODO: for each parameter available in the register, try to look if this parameter is available
		
		const ParameterDoc doc(*it);

		const std::string paramLocation(root+"/"+doc.name);
		
		if (hasParam(paramLocation))
			params[doc.name] = getParam<std::string>(paramLocation, doc.defaultValue);
	}
	return desc->createInstance(params);
}

template<typename R>
void initModulesFromRegistrar(const std::string& paramNamespace, const R& registrar, PointMatcherSupport::SharedPtrVector<typename R::TargetType>& modules)
{
	const int count(getParam<int>(paramNamespace + "Count", 0));
	for (int i = 0; i < count; ++i)
	{
		const std::string root((boost::format("%1%/%2%") % paramNamespace % i).str());
		modules.push_back(initModuleFromRegistrar(root, registrar));
	}
}

void populateParametersBase(PM::ICPChainBase& icp)
{
	// icp parameters
	PM pm;
	
	initModulesFromRegistrar("readingDataPointsFilters", pm.REG(DataPointsFilter), icp.readingDataPointsFilters);
	//initModulesFromRegistrar("readingStepDataPointsFilters", pm.REG(DataPointsFilter), icp.readingStepDataPointsFilters);
	initModulesFromRegistrar("keyframeDataPointsFilters", pm.REG(DataPointsFilter), icp.keyframeDataPointsFilters);
	initModulesFromRegistrar("transformations", pm.REG(Transformation), icp.transformations);
	icp.matcher.reset(initModuleFromRegistrar("matcher", pm.REG(Matcher)));
	initModulesFromRegistrar("outlierFilters", pm.REG(OutlierFilter), icp.outlierFilters);
	icp.errorMinimizer.reset(initModuleFromRegistrar("errorMinimizer", pm.REG(ErrorMinimizer)));
	initModulesFromRegistrar("transformationCheckers", pm.REG(TransformationChecker), icp.transformationCheckers);
	icp.inspector.reset(initModuleFromRegistrar("inspector", pm.REG(Inspector)));
	PointMatcherSupport::setLogger(initModuleFromRegistrar("logger", pm.REG(Logger)));
	// FIXME: consistency check?
}

void populateParameters(PM::ICPSequence& icp)
{
	icp.ratioToSwitchKeyframe = getParam("ratioToSwitchKeyframe", 0.8);
	
	populateParametersBase(icp);
}

#endif // __ICP_CHAIN_CREATION_H
