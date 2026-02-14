/*
    Copyright 2009-14 Stephan Martin, Dominik Gummel

    This file is part of Multidisplay.

    Multidisplay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Multidisplay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Multidisplay.  If not, see <http://www.gnu.org/licenses/>.

    MultiDisplayDefinesDigifant.h
    Created on: 31.07.2014
*/

/**
 * @file MultidisplayDefinesDigifant.h
 * @brief ECU-specific configuration for VW Digifant-1 engine management.
 *
 * This header is included by MultidisplayDefines.h when DIGIFANT is defined.
 * The Digifant-1 ECU is commonly found in VW/Audi 2.0L 8-valve turbo
 * applications (e.g. Audi S2).
 *
 * Key features:
 *   - K-line serial diagnostic data stream (4-cylinder knock retard,
 *     ignition advance, boost, ECT, IAT, RPM)
 *   - Optional DK poti (throttle potentiometer) or idle/WOT switch
 *   - Uses the Digifant internal MAP sensor by default
 */

#ifndef MULTIDISPLAYDEFINESDIGIFANT_H_
#define MULTIDISPLAYDEFINESDIGIFANT_H_

/* ========================================================================== */
/*  Digifant K-Line & Throttle Input                                          */
/* ========================================================================== */
/*
 * DIGIFANT_KLINE   – enable the Digifant serial K-line data receiver.
 *                     Receives 34-byte frames at 6667 baud from the ECU.
 * DIGIFANT_DK_POTI – the throttle is read from a potentiometer (0-5 V).
 *                     If NOT defined, only the idle/WOT switch pair is used.
 */
#define DIGIFANT_KLINE
#define DIGIFANT_DK_POTI

/* ========================================================================== */
/*  Boost MAP Sensor Selection (choose one)                                   */
/* ========================================================================== */
/*
 * USE_DIGIFANT_MAPSENSOR – read boost from the Digifant's own MAP sensor.
 * See MultidisplayDefinesVR6.h for descriptions of alternative sensors.
 */
#define USE_DIGIFANT_MAPSENSOR
/* #define BOOST_PLX_SMVACBOOST */
/* #define BOOST_MOTOROLA_MPX4250 */
/* #define BOOST_FREESCALE_MPXA6400A */
/* #define BOOST_BOSCH_200KPA */

/* ========================================================================== */
/*  Lambda (O₂) Sensor Selection                                             */
/* ========================================================================== */
#define LAMBDA_WIDEBAND
#define LAMBDA_PLX_SMAFR
/* #define LAMBDA_INNOVATE_LC1 */
/* #define LAMBDASTANDALONE A2 */

/* ========================================================================== */
/*  RPM Signal Configuration                                                  */
/* ========================================================================== */
/*
 * USE_DIGIFANT_RPM – RPM is extracted from the Digifant K-line data stream.
 * RPMFACTOR 2.34  – calibration for 4-cylinder engine with 75 kΩ pull-up.
 */
/* #define USE_RPM_ONBOARD */
#define USE_DIGIFANT_RPM
#define RPMFACTOR 2.34
/* #define RPMFACTOR 5.21 */               /* 20 kΩ pull-up alternative */

/* ========================================================================== */
/*  Vehicle Speed Configuration                                               */
/* ========================================================================== */
#define SPEEDFACTOR 0.38
#define SPEEDCORRECTIONFACTOR 0.9
/* #define SPEEDFACTOR 0.390625 */
/* #define SPEEDCORRECTIONFACTOR 1 */

/* ========================================================================== */
/*  Type-K EGT Probes                                                         */
/* ========================================================================== */
/*
 * 4-cylinder Digifant engines typically have 4 or 5 EGT probes.
 * (one per exhaust runner + optionally one in the downpipe)
 */
#define NUMBER_OF_ATTACHED_TYPE_K 5

#endif /* MULTIDISPLAYDEFINESDIGIFANT_H_ */
