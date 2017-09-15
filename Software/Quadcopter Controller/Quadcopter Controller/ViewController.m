//
//  ViewController.m
//  Quadcopter Controller
//
//  Created by Yongyang Nie on 3/4/17.
//  Copyright © 2017 Yongyang Nie. All rights reserved.
//

#import "ViewController.h"

@interface ViewController ()

@end

@implementation ViewController

#pragma BLEManager Delegate

- (IBAction)emergencyLanding:(id)sender {
    [self.manager sendData:1 data:1200];
}

-(void)didDisconnect:(NSError *)error{
    [self showAlert:@"Fail to connect!" message:[NSString stringWithFormat:@"Fail to connect, error:%@", error.localizedDescription]];
}

-(void)BLEManagerFeedback:(NSString *)feedback{
    NSLog(@"%@", feedback);
}

-(void)didConnectToPeripheral:(CBPeripheral *)peripheral{
    NSLog(@"connected: %@", peripheral.name);
    [self showAlert:@"Connected!" message:@"Successfully connected to quadcopter"];
    [self.manager sendData:1 data:1200];
}

#pragma mark - Private

-(void)showAlert:(NSString *)title message:(NSString *)message{
    UIAlertController *alert = [UIAlertController alertControllerWithTitle:title message:message preferredStyle:UIAlertControllerStyleAlert];
    UIAlertAction *action = [UIAlertAction actionWithTitle:@"Ok" style:UIAlertActionStyleCancel handler:nil];
    [alert addAction:action];
    [self presentViewController:alert animated:YES completion:nil];
}

- (IBAction)quickConnect:(id)sender {
    
    self.manager = [[BLEManager alloc] init];
    self.manager.delegate = self;
}

-(void)rightPan:(UIPanGestureRecognizer *)pan{

    if (pan.state == UIGestureRecognizerStateEnded) {
        [UIView animateWithDuration:0.2 animations:^{
            self.rightStick.center = self.rightPosition;
        }];
    }else if (pan.state == UIGestureRecognizerStateChanged){
        CGPoint location = [pan locationInView:self.view];
        if (location.y < 95 || location.y > 220) {
            location.y = self.rightStick.center.y;
        }
        if (location.x < 355 || location.x > 490){
            location.x = self.rightStick.center.x;
        }
        self.rightStick.center = location;
        NSLog(@"%f", self.rightStick.center.y);
    }
}

-(void)leftPan:(UIPanGestureRecognizer *)pan{
    
    if (pan.state == UIGestureRecognizerStateEnded) {
        [UIView animateWithDuration:0.2 animations:^{
            self.leftStick.center = self.leftPosition;
        }];
    }else if (pan.state == UIGestureRecognizerStateChanged){
        CGPoint location = [pan locationInView:self.view];
        if (location.y < 95 || location.y > 220) {
            location.y = self.leftStick.center.y;
        }
        if (location.x < 80 || location.x > 215){
            location.x = self.leftStick.center.x;
        }
        self.leftStick.center = location;
        NSLog(@"%f", self.leftStick.center.x);
    }
}

-(void)viewDidAppear:(BOOL)animated{
    
    self.rightPosition = self.rightStick.center;
    self.leftPosition = self.leftStick.center;
    [super viewDidAppear:YES];
}

- (void)viewDidLoad {
    [super viewDidLoad];
    
    UIPanGestureRecognizer *rightPan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(rightPan:)];
    UIPanGestureRecognizer *leftPan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(leftPan:)];
    
    [self.leftStick addGestureRecognizer:leftPan];
    [self.rightStick addGestureRecognizer:rightPan];
    
    self.leftStick.layer.cornerRadius = 5;
    self.rightStick.layer.cornerRadius = 5;
    self.leftStick.layer.masksToBounds = YES;
    self.rightStick.layer.masksToBounds = YES;
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
