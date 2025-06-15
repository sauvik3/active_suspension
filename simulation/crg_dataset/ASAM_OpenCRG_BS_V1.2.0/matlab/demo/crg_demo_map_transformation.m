% CRG_DEMO_MAP_TRANSFORMATION CRG demo to perform transformations.
%   CRG_DEMO_MAP_TRANSFORMATION demonstrates how mpro projection data can
%   be used to perform geodetic transformations.
%   A single point in WGS84 coordinates (Latitude, Longitude, Height)
%   is transformed to common map projections (UTM, GK).
%
%   Example:
%   crg_demo_map_transformation         runs this demo

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               demo
% file name:             crg_demo_map_transformation.m
% author:                ASAM e.V.
%
%
% C by ASAM e.V., 2020
% Any use is limited to the scope described in the license terms.
% The license terms can be viewed at www.asam.net/license
%
% More Information on ASAM OpenCRG can be found here:
% https://www.asam.net/standards/detail/opencrg/
%
% *****************************************************************

%% Demo proceedings
%
% * UTM (UTM2BLH, BLH2UTM)
% * GK (GK2BLH, BLH2GK)
% * .. 
%

% DEFAULT SETTINGS
% clear enviroment
clear all;
close all;
clc;

% display format
format long g;

% example position ASAM e.V. 
% (Altlaufstraße 40, 85635 Höhenkirchen-Siegertsbrunn)
org_llh = [	48.02331, 11.71584, 584.0]; % WGS84

%% Test1 (UTM)
fprintf('%% Test1 (UTM)')
% create mpro
mpro.gell.nm='WGS84';   % global datum
mpro.proj.nm='UTM_32U'; % local datum

% WGS84 llh degree -> WGS84 llh radian
llh = [pi/180*org_llh(1), pi/180*org_llh(2), org_llh(3)];

% transform WGS84 llh radian -> UTM_32U
enh_utm = map_geod2pmap_tm(llh, mpro.gell, mpro.proj)

% transform UTM_32U -> WGS84 llh radian
llh = map_pmap2geod_tm(enh_utm,  mpro.gell, mpro.proj);

% WGS84 llh radian -> WGS84 llh degree
llh = [180/pi*llh(1), 180/pi*llh(2), llh(3)]

%% Test2 (GK with datum transformation)
fprintf('%% Test2 (GK with datum transformation)')
% create mpro2
mpro2.gell.nm='WGS84';
mpro2.lell.nm='BESSELDHDN';
mpro2.proj.nm='GK3_4';
mpro2.tran.nm='HN7';     % transformation
% 7 Parameter Helmerttransformation (example for Bavaria from LDBV)
mpro2.tran.ds = -5.2379 * 0.000001;
mpro2.tran.rx = (0.7201 / 3600) * (pi / 180);
mpro2.tran.ry = (0.1112 / 3600) * (pi / 180);
mpro2.tran.rz = (-1.7209 / 3600) * (pi / 180);
mpro2.tran.tx = -604.7365;
mpro2.tran.ty = -72.3946;
mpro2.tran.tz = -424.402;
mpro2=map_check(mpro2);

% WGS84 llh degree -> WGS84 llh radian
llh = [pi/180*org_llh(1), pi/180*org_llh(2), org_llh(3)];

% transform WGS84 llh radian -> GK3 zone 4 (BESSELDHDN)
% transformation includes datum transformation, see map_global2plocal.m
enh_gk = map_global2plocal(llh, mpro2)

% transform GK3 zone 4 (BESSELDHDN) -> WGS84 llh radian
% transformation includes datum transformation, see map_global2plocal.m
llh = map_plocal2global(enh_gk, mpro2);

% WGS84 llh radian -> WGS84 llh degree
llh = [180/pi*llh(1), 180/pi*llh(2), llh(3)]


%% Test3 (GK to UTM)
fprintf('%% Test3 (GK to UTM)')
% transform GK3 zone 4 (BESSELDHDN) -> WGS84 llh radian
% transformation includes datum transformation, see map_global2plocal.m
llh = map_plocal2global(enh_gk, mpro2);

% transform WGS84 llh radian -> UTM_32U
enh_utm = map_geod2pmap_tm(llh, mpro.gell, mpro.proj)
