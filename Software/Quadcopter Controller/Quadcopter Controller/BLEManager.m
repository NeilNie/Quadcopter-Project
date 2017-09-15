
//
//  BLEManager.m
//  Quadcopter Controller
//
//  Created by Yongyang Nie on 4/17/17.
//  Copyright © 2017 Yongyang Nie. All rights reserved.
//

#import "BLEManager.h"

@implementation BLEManager

- (instancetype)init
{
    self = [super init];
    if (self) {
        self.cm = [[CBCentralManager alloc] initWithDelegate:self queue:dispatch_queue_create("com.yongyang.controller.bluetooth", 0)];
        self.status = disconnected;
        self.cm.delegate = self;
    }
    return self;
}

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
    
    switch (central.state) {
        case CBManagerStatePoweredOff:{
            [self.delegate BLEManagerFeedback:@"CBManager Error: bluetooth is powered off"];
            break;
        }
        case CBManagerStatePoweredOn:{
            [self startScanning];
            break;
        }
        case CBManagerStateResetting:{
            break;
        }
        default:
            break;
    }
}

-(void)centralManager:(CBCentralManager *)central didDisconnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error{
    self.status = disconnected;
    [self.delegate didDisconnect:error];
    self.peripheral = nil;
}

-(void)centralManager:(CBCentralManager *)central didConnectPeripheral:(CBPeripheral *)peripheral{
    
    if ([peripheral.identifier.UUIDString isEqualToString:@"D7AC59DF-3F15-45CB-AEBC-1B19199EBD22"]) {
        self.status = connected;
        [self.delegate didConnectToPeripheral:peripheral];
        self.peripheral.delegate = self;
        [self.peripheral discoverServices:nil];
    }
}

-(void)centralManager:(CBCentralManager *)central didFailToConnectPeripheral:(CBPeripheral *)peripheral error:(NSError *)error{
    self.status = disconnected;
    self.peripheral = nil;
    [self.delegate didDisconnect:error];
}

-(void)centralManager:(CBCentralManager *)central didDiscoverPeripheral:(CBPeripheral *)peripheral advertisementData:(NSDictionary<NSString *,id> *)advertisementData RSSI:(NSNumber *)RSSI{
    
    if ([peripheral.identifier.UUIDString isEqualToString:@"D7AC59DF-3F15-45CB-AEBC-1B19199EBD22"] && self.status == disconnected) {
        NSLog(@"discovered device");
        self.status = connecting;
        [self connectToPeripheral:peripheral];
    }
}

#pragma mark - CBPeripheral delegate

-(void)peripheral:(CBPeripheral *)peripheral didDiscoverCharacteristicsForService:(CBService *)service error:(NSError *)error{
    
    for (CBCharacteristic *characteristic in [service characteristics]) {
        if ([[characteristic UUID].UUIDString isEqualToString:@"6E400002-B5A3-F393-E0A9-E50E24DCCA9E"]) {
            self.characteristic = characteristic;
            [self.delegate BLEManagerFeedback:@"characteristic discovered"];
        }
    }
}

- (void)peripheral:(CBPeripheral *)peripheral didDiscoverServices:(NSError *)error {
    
    if (error) {
        NSLog(@"Error %@\n", error);
        return;
    }
    
    NSArray *services = [peripheral services];
    if (services.count == 0) {
        NSLog(@"No Services");
        return;
    }
    
    for (CBService *service in services) {
        if ([[service UUID].UUIDString isEqualToString:@"6E400001-B5A3-F393-E0A9-E50E24DCCA9E"]) {
            [self.peripheral discoverCharacteristics:nil forService:service];
        }
    }
}

#pragma mark - Private

-(void)connectToPeripheral:(CBPeripheral *)peripheral{
    
    [self.cm stopScan];
    NSLog(@"stopped scanning");
    [self.cm connectPeripheral:peripheral options:@{CBConnectPeripheralOptionNotifyOnDisconnectionKey: [NSNumber numberWithBool:YES]}];
    NSLog(@"begin connecting");
    NSLog(@"%@", peripheral.name);
    self.peripheral = peripheral;
}

-(void)startScanning{
    
    NSLog(@"started scanning");
    //Check if Bluetooth is enabled
    if (self.cm.state == CBManagerStatePoweredOff) {
        NSLog(@"bluetooth is off");
        return;
    }
    [self.cm scanForPeripheralsWithServices:nil options:@{CBCentralManagerScanOptionAllowDuplicatesKey: @YES}];
}

-(void)sendData:(UInt16)channel data:(UInt16)position{
    
    UInt16 i = channel * 10000 + position;
    NSData *data = nil;
    data = [NSData dataWithBytes:&i length:sizeof(i)];
    if (self.characteristic) {
        [self.peripheral writeValue:data forCharacteristic:self.characteristic type:CBCharacteristicWriteWithResponse];
    }else{
        [self.delegate BLEManagerFeedback:@"no characteristic"];
    }
    
}

@end
