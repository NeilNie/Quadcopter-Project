//
//  BLEPeripheral.h
//  Quadcopter Controller
//
//  Created by Yongyang Nie on 4/17/17.
//  Copyright © 2017 Yongyang Nie. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <CoreBluetooth/CoreBluetooth.h>

@interface BLEPeripheral : NSObject <CBPeripheralDelegate>

@property (strong, nonatomic) CBPeripheral *peripheral;
@property (strong, nonatomic) CBCharacteristic *characteristic;

@end
