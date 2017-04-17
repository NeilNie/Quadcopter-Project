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

-(void)BLEManagerFeedback:(NSString *)feedback{
    NSLog(@"feedback");
}

-(void)didConnectToPeripheral:(CBPeripheral *)peripheral{
    NSLog(@"connected: %@", peripheral.name);
    UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Connected!" message:[NSString stringWithFormat:@"Did connect to %@", peripheral.name] preferredStyle:UIAlertControllerStyleAlert];
    UIAlertAction *action= [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:nil];
    [alert addAction:action];
    [self showViewController:alert sender:nil];
}

#pragma mark - Private

-(void)rightPan:(UIPanGestureRecognizer *)pan{

    if (pan.state == UIGestureRecognizerStateEnded) {
        [UIView animateWithDuration:0.2 animations:^{
            self.rightStick.center = self.rightPosition;
        }];
    }else if (pan.state == UIGestureRecognizerStateChanged){
        CGPoint location = [pan locationInView:self.view];
        if (location.y < 95 || location.y > 220) {
            location.y = self.rightStick.center.y;
        }else if (location.x < 355 || location.x > 490){
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
        }else if (location.x < 80 || location.x > 215){
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
    
    self.manager = [[BLEManager alloc] init];
    self.manager.delegate = self;
    //[self.manager startScanning];
    // Do any additional setup after loading the view, typically from a nib.
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
