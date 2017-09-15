//
//  BLEManager.h
//  Quadcopter Controller
//
//  Created by Yongyang Nie on 4/17/17.
//  Copyright © 2017 Yongyang Nie. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>
#import <UIKit/UIKit.h>
#import "BLEPeripheral.h"

#define CHAR_UIID		"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

enum ConnectionStatus{
    idle = 0,
    connected,
    disconnected,
    connecting
};

@interface BLEManager : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property (retain, nonatomic) id delegate;
@property (strong, nonatomic) CBCentralManager *cm;
@property (strong, nonatomic) CBPeripheral *peripheral;
@property (strong, nonatomic) CBCharacteristic *characteristic;
@property enum ConnectionStatus status;

-(void)startScanning;
-(void)sendData:(UInt16)channel data:(UInt16)position;

@end

@protocol BLEManagerDelegate <NSObject>

-(void)BLEManagerFeedback:(NSString *)feedback;
-(void)didConnectToPeripheral:(CBPeripheral *)peripheral;
-(void)didDisconnect:(NSError *)error;

@end
