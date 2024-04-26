#ifndef CAN_DRIVER_CONFIG_H
#define CAN_DRIVER_CONFIG_H

// Comment out the driver that is not being used

#if !defined(MCP_DRIVER) || !defined(STM_DRIVER)
//#define MCP_DRIVER 1
#define STM_DRIVER 1
#endif // MCP_DRIVER) || STM_DRIVER

#endif /* CAN_DRIVER_CONFIG_H */