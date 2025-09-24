add_cia402_library(
    NAME EthercatManager  
    SOURCES src/EthercatManager.cpp
    PUBLIC_HEADERS include/CiA402/EthercatManager.h include/CiA402/LogComponent.h
    PUBLIC_LINK_LIBRARIES soem YARP::YARP_os 
)
