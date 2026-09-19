
#include <TargetConditionals.h>
#if TARGET_OS_IOS == 1 || TARGET_OS_VISION == 1
#import <UIKit/UIKit.h>
#else
#import <Cocoa/Cocoa.h>
#endif

#define IPLUG_AUVIEWCONTROLLER IPlugAUViewController_vSpectralDelay
#define IPLUG_AUAUDIOUNIT IPlugAUAudioUnit_vSpectralDelay
#import <SpectralDelayAU/IPlugAUViewController.h>
#import <SpectralDelayAU/IPlugAUAudioUnit.h>

//! Project version number for SpectralDelayAU.
FOUNDATION_EXPORT double SpectralDelayAUVersionNumber;

//! Project version string for SpectralDelayAU.
FOUNDATION_EXPORT const unsigned char SpectralDelayAUVersionString[];

@class IPlugAUViewController_vSpectralDelay;
