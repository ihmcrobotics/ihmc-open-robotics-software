#!/bin/sh

#This script loads up all the webpages with .java code snippets and checks for errors (this is only for webpages with existing source code files)
#Note: the SCS API webpages haven't been added to this yet

urls=( https://ihmcroboticsdocs.github.io/ihmc-open-robotics-software/docs/03-new-class-eclipse.html
https://ihmcroboticsdocs.github.io/ihmc-open-robotics-software/docs/03-new-class-intellij.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/02-simple-pendulum-simulation.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/03-simple-pendulum-robot.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/01-adding-control-to-a-simulation.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/01-creating-links.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/02-mobile-simulation.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/03-mobile-robot.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/04-mobile-robot-description.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/02-falling-brick-setup.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/02-flyball-governor-simulation.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/03-flyball-governor-robot.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/04-flyball-controller.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/05-flyball-governor-common-control-parameters.html
https://ihmcroboticsdocs.github.io/simulation-construction-set/docs/06-description-and-analysis.html)

exitCode=0

for (( i = 0; i < ${#urls[@]}; ++i )); do
	echo "Checking ${urls[i]}"
	node snippetErrors.js ${urls[i]} || exitCode=1
done

exit $exitCode
  
