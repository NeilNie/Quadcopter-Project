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

enum ConnectionStatus{
    idle = 0,
    connected,
    disconnected,
    connecting
};

@interface BLEManager : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>

@property (retain, nonatomic) id delegate;
@property (strong, nonatomic) CBCentralManager *cm;
@property (strong, nonatomic) CBPeripheral *connecting;
@property enum ConnectionStatus status;

-(void)startScanning;

@end

@protocol BLEManagerDelegate <NSObject>

-(void)BLEManagerFeedback:(NSString *)feedback;
-(void)didConnectToPeripheral:(CBPeripheral *)peripheral;
-(void)didDisconnect:(NSError *)error;

@end
