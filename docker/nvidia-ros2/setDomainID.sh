#!/bin/bash

setDomainID () {
   domainID=$1
   export ROS_DOMAIN_ID=$domainID
   echo "export ROS_DOMAIN_ID=$domainID" >> /home/robotlab/.bashrc
}
