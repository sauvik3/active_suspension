%% Usage of CRG_ISEQUAL
% Introducing the usage of crg_isequal.
% Examples are included for a set of common CRG-file formats.
% The file comments are optimized for the matlab publishing makro.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               test
% file name:             crg_test_isequal.m
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

%% Test proceedings
%
% * load demo/real crg-file
% * modify crg-structure
% * compare
% * display result
%

% DEFAULT SETTINGS
% clear enviroment
clear all;
close all;
% display results
dispRes = 1;

%% Test1 ( check crg_b2z )

mdat = crg_read('demo7.crg');

data = mdat;
data = crg_b2z(mdat);

[crgEqual, dd] = crg_isequal(mdat, data);

if dispRes, crg_show_isequal(dd); end

%% Test1.1 ( check crg_s2z )

mdat = crg_read('demo8.crg');

data = mdat;
data = crg_s2z(mdat);

[crgEqual, dd] = crg_isequal(mdat, data);

if dispRes, crg_show_isequal(dd); end

%% Test1.2 ( check crg_b2z & crg_s2z)

mdat = crg_read('demo8.crg');

data = mdat;
data = crg_s2z(mdat);
data = crg_b2z(mdat);

[crgEqual, dd] = crg_isequal(mdat, data);

if dispRes, crg_show_isequal(dd); end

%% Test2 ( cut -> concat -> compare )
data = crg_read('demo1.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 501]);
crg2 = crg_cut_iuiv(crg0, [500, 1000]);

crg3 = crg_append(crg1, crg2);

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test2.1 ( cut -> concat -> compare )
data = crg_read('demo2.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 501]);
crg2 = crg_cut_iuiv(crg0, [500, 1000]);

crg3 = crg_append(crg1, crg2);

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test2.2  ( cut -> concat -> compare )
data = crg_read('demo3.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 501]);
crg2 = crg_cut_iuiv(crg0, [500, 1000]);

crg3 = crg_append(crg1, crg2);

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end


%% Test2.3 ( cut -> concat -> compare )
data = crg_read('demo4.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 501]);
crg2 = crg_cut_iuiv(crg0, [500, 1000]);
crg3 = crg_append(crg1, crg2);

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test2.4 ( cut -> concat -> compare )
data = crg_read('demo5.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 501]);
crg2 = crg_cut_iuiv(crg0, [500, 1000]);

crg3 = crg_append(crg1, crg2);

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test2.5 ( cut -> concat -> compare )
data = crg_read('demo6.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 501]);
crg2 = crg_cut_iuiv(crg0, [500, 1000]);

crg3 = crg_append(crg1, crg2);

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test2.6 ( cut -> concat -> compare )
data = crg_read('demo7.crg');
crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 301]);
crg2 = crg_cut_iuiv(crg0, [300, 1000]);

crg3  = crg_append(crg1, crg2 );

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test2.7 ( cut -> concat -> compare )
data = crg_read('demo8.crg');

crg0 = crg_cut_iuiv(data, [1, 1000]);
crg1 = crg_cut_iuiv(crg0, [1, 301]);
crg2 = crg_cut_iuiv(crg0, [300, 1000]);

crg3 = crg_append(crg1, crg2 );

[crgEqual, dd] = crg_isequal(crg0, crg3);

if dispRes, crg_show_isequal(dd); end

%% Test3 ( check crg_ext_banking )

data = crg_read('demo7.crg');

exdata = crg_ext_banking(data);

[crgEqual, dd] = crg_isequal(data, exdata);

if dispRes, crg_show_isequal(dd); end

%% Test3.1 (check crg_ext_slope )

data = crg_read('demo8.crg');

exdata = crg_ext_slope(data, 0.001);

[crgEqual, dd] = crg_isequal(data, exdata);

if dispRes, crg_show_isequal(dd); end

%% Test3.2 (check crg_ext_slope/banking)

data = crg_read('demo8.crg');

exdata = crg_ext_banking(data, 0.001);
exdata = crg_ext_slope(exdata, 0.001);

[crgEqual, dd] = crg_isequal(data, exdata);

if dispRes, crg_show_isequal(dd); end

%% Test4 (check rerender)

data = crg_read('demo1.crg');

data = crg_rerender(data, [0.02 0.02]);

dat = crg_rerender(data, [0.01 0.01]);
dat = crg_rerender(dat, [0.02 0.02]);

[crgEqual, dd] = crg_isequal(data, dat);

if dispRes, crg_show_isequal(dd); end

%% Test4.1 (check rerender)

data = crg_read('demo8.crg');

data = crg_rerender(data, [0.01 0.01]);

dat = crg_rerender(data, [0.02 0.02]);
dat = crg_rerender(dat, [0.01 0.01]);

dat = crg_cut_iuiv(dat, [1 size(data.z,1)]);
data = crg_cut_iuiv(data, [1 size(data.z,1)], [1 size(dat.z,2)]);

[crgEqual, dd] = crg_isequal(data, dat);

if dispRes, crg_show_isequal(dd); end

%% Test4.2 (check rerender)

data = crg_read('../crg-bin/belgian_block.crg');

data = crg_rerender(data, [0.01 0.01]);

dat = crg_rerender(data, [0.005 0.005]);
dat = crg_rerender(dat, [0.01 0.01]);

dat = crg_cut_iuiv(dat, [1 size(data.z,1)]);
data = crg_cut_iuiv(data, [1 size(data.z,1)], [1 size(dat.z,2)]);

[crgEqual, dd] = crg_isequal(data, dat);

if dispRes, crg_show_isequal(dd); end

%% Test5 ( check real crg_ext_slope/banking )

dat = crg_read('../crg-bin/belgian_block.crg');

exdata = crg_ext_banking(dat, 0.0000000000003);
exdata = crg_ext_slope(exdata);

[crgEqual, dd] = crg_isequal(dat, exdata);

if dispRes, crg_show_isequal(dd); end
