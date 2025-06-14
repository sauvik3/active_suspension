function [y] = smooth_firfilt(x, w, opts)
% SMOOTH_FIRFILT Smooth input signals with symmetric FIR filter.
%   Y = SMOOTH_FIRFILT(X, W, OPTS) filters the input signals using
%   symmetric moving average (and some relatives of it) FIR filtering.
%
%   Inputs:
%   X       Data to be filtered. If X is a matrix, SMOOTH_FILTER operates
%           on the columns of X.
%   W       FIR filter window width on each side of the center value.
%   OPTS    (optional) stuct array with
%       .reflect    reflection mode to extrapolate input data at begin and
%                   end to have start-up and ending transients as desired.
%                   'line' extrapolates by (line) reflection at begin and
%                   end of data (default).
%                   'point' extrapolates by point reflection at begin and
%                   end of data.
%                   'const' extrapolates with constant value at begin and
%                   end of data.
%                   'zero' extrapolates with zero value at begin and end of
%                   data.
%       .window     filter window type (see also at
%                   http://en.wikipedia.org/wiki/Window_function) All
%                   windows are constructed to have non-zero end points.
%                   'rectangular' generates a moving average (default).
%                   'triangular' uses a triangular window.
%                   'hann' uses a hann window.
%                   'sine' uses a sine window.
%
%   Outputs:
%   Y       Filtered data.
%
%   Examples:
%   y = smooth_firfilt(x, 10)
%       Generates results using symmetric moving average of 2*10+1 input
%       values using (line) reflection at begin and end of input data.
%   opts = struct
%   opts.reflect = 'point';
%   opts.window = 'triangular';
%   y = smooth_firfilt(x, 10, opts)
%       Generates results using symmetric triangular weithted average of
%       2*10+1 input values using poiint reflection at begin and end of
%       input data.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             smooth_firfilt.m 
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

%% check inputs

if isempty(w) || length(w) > 1 || ~isnumeric(w) || ~isreal(w) || w~=round(w) || w<=0,
    error('Filter width must be a real, positive integer.')
end

if nargin < 3
    opts = struct;
end


%% recursive call for matrix input
% operate on columns as MATLAB filter function would do

[m,n] = size(x);
if (n>1) && (m>1)
    y = x;
    for i = 1:n
        y(:,i) = smooth_firfilt(x(:,i), w, opts);
    end
    return
end

%% handle input column vectors

if n==1
    x = x.'; % convert column to row
end

if length(x) <= w
    error('Data must have length greater than filter width.');
end

reflect = 'line';
if isfield(opts, 'reflect')
   reflect = opts.reflect;
end

window = 'rectangular';
if isfield(opts, 'window')
    window = opts.window;
end

% extend input vector by filter width

switch(reflect)
    case('line')
        xx = [       x(w+1:-1:2) x          x(end-1:-1:end-w)];
    case('point')
        xx = [2*x(1)-x(w+1:-1:2) x 2*x(end)-x(end-1:-1:end-w)];
    case('const')
        xx = [x(1)*ones(1,w)     x           x(end)*ones(1,w)];
    case('zero')
        xx = [zeros(1,w)         x                 zeros(1,w)];
    otherwise
        error('illegal opts.reflect value')
end

%% build symmetric filter

switch(window)
    case('rectangular')
        f = ones(1, w+1);
    case('triangular')
        f = 1:w+1;
    case('hann')
        f = 1 + cos((-w:0)*(pi/(w+1)));
    case('sine')
        f = sin((1:w+1)*(pi/2/(w+1)));
    otherwise
        error('illegal opts.window value')
end
f = [f f(end-1:-1:1)]; % line reflect filter coefficients

%% normalize filter

f = f / sum(f);

%% apply filter

y = zeros(size(x));
for i = 1:length(f)
    y = y + f(i)*xx(i:i-1+length(x));
end

%% convert output to column if input was column

if n == 1
    y = y(:);
end

end
