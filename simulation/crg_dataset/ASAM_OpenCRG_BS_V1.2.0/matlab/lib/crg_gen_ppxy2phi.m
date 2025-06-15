function [data, err] = crg_gen_ppxy2phi(ppxy, uinc, opts)
% CRG_GEN_PPXY2PHI Generate reference line heading from polynomial.
%   [DATA, ERR] = CRG_GEN_PPXY2PHI(PPXY, UINC, OPTS) generates a
%   partial OpenCRG struct with reference line heading information by evaluating
%   the given smooth polynomial in pp-form.
%
%   Inputs:
%   PPXY    complex piecewise polynomial (e.g. spline) in pp-form
%           describing the smoothed reference line.
%   UINC    reference line increment
%   OPTS    struct for method options (optional, no default)
%   .meth   selects discretization method (optional, default: 8)
%           The default method should always work, the others are left
%           over from development to allow further discussions.
%           (description of methods in subfunction codes below)
%
%   Outputs:
%   DATA    partial OpenCRG data structure (see CRG_INTRO) consisting of
%           DATA.u
%           DATA.p
%           DATA.head.ubeg
%           DATA.head.uend
%           DATA.head.uinc
%           DATA.head.pbeg
%           DATA.head.pend
%           DATA.head.xbeg
%           DATA.head.ybeg
%           DATA.head.xend
%           DATA.head.yend
%   ERR     position error after forward integration from start to end
%           using the single precision P values.
%
%   Examples:
%   [data, err] = crg_crg_gen_ppxy2phi(ppxy, uinc)
%       Generates a CRG reference line.
%
%   See also CRG_GEN_PXY2PPXY, CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_gen_ppxy2phi.m 
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

%% set defaults for optional arguments

if nargin < 3
    opts.meth = 8; % default method
end

%% select method

switch opts.meth
    case 0
        [data, err] = ppxy2p0(ppxy, uinc);
    case 1
        [data, err] = ppxy2p1(ppxy, uinc);
    case 2
        [data, err] = ppxy2p2(ppxy, uinc);
    case 3
        [data, err] = ppxy2p3(ppxy, uinc);
    case 4
        [data, err] = ppxy2p4(ppxy, uinc);
    case 5
        [data, err] = ppxy2p5(ppxy, uinc);
    case 6
        [data, err] = ppxy2p6(ppxy, uinc);
    case 7
        [data, err] = ppxy2p7(ppxy, uinc);
    case 8
        [data, err] = ppxy2p8(ppxy, uinc);
    otherwise
        error('CRG:genError', 'illegal argument opts.meth=%d', opts.meth)
end

end

%% meth = 0

function [data, err] = ppxy2p0(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% evaluate spline with fixed s interval

r = ppval(ppxy, s0:uinc:se+uinc);
p = single(angle(diff(r)));

% calculate integration error for last point

r1 = r(end); % by spline evaluation at last position
r2 = r(1) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r(1));
data.head.ybeg = imag(r(1));
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 1

function [data, err] = ppxy2p1(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% loop over spline with fixed s interval
% same result as ppxy2p0, but slower

r0 = ppval(ppxy, s0);
np = 0;

r1 = r0;
for s1 = s0+uinc : uinc : se+uinc
    np = np + 1;

    r2 = ppval(ppxy, s1);

    p(np) = single(angle(r2-r1));
    r1 = r2;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 2

function [data, err] = ppxy2p2(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with fixed s interval
% same result as ppxy2p1, but slower

r0 = ppval(ppxy, s0);
np = 0;

s1 = s0;
while s1 < se
    np = np + 1;

    r1 = ppval(ppxy, s1+uinc);

    p(np) = single(angle(r1-r0));
    r0 = r1;
    s1 = s1 + uinc;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 3

function [data, err] = ppxy2p3(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with adapted s interval
% local prediction with uinc
% does improve err compared to ppxy2p2

r0 = ppval(ppxy, s0);
np = 0;

r1 = r0;
s1 = s0;
while s1 < se
    np = np + 1;

    r2 = ppval(ppxy, s1+uinc);
    d1 = r2-r1;

    f  = uinc/abs(d1);
    p(np) = single(angle(d1));
    r1 = r1 + f*d1;
    s1 = s1 + f*uinc;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 4

function [data, err] = ppxy2p4(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with adapted s interval
% local prediction with last s interval
% err is similar to ppxy2p3

r0 = ppval(ppxy, s0);
np = 0;
ds = uinc;

r1 = r0;
s1 = s0;
while s1 < se
    np = np + 1;

    r2 = ppval(ppxy, s1+ds);
    d1 = r2-r1;

    f  = uinc/abs(d1);
    ds = f*ds;
    p(np) = single(angle(d1));
    r1 = r1 + f*d1;
    s1 = s1 + ds;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end%% meth = 5

%% meth = 5

function [data, err] = ppxy2p5(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with adapted s interval
% local prediction with last s interval and iteration
% err is similar to ppxy2p4

r0 = ppval(ppxy, s0);
np = 0;
ds = uinc;

r1 = r0;
s1 = s0;
while s1 < se
    np = np + 1;

    for j = 1:10 % assume convergence after 10 iterations
        r2 = ppval(ppxy, s1+ds);

        d1 = r2-r1;

        f  = uinc/abs(d1);
        ds = f*ds;
        if abs(f-1) < 1e-10, break, end
    end
    p(np) = single(angle(d1));
    r1 = r1 + f*d1;
    s1 = s1 + ds;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 6

function [data, err] = ppxy2p6(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with adapted s interval
% local prediction with uinc
% position update by effective single angle
% does improve err compared to ppxy2p5

r0 = ppval(ppxy, s0);
np = 0;

r1 = r0;
s1 = s0;
while s1 < se
    np = np + 1;

    r2 = ppval(ppxy, s1+uinc);
    d1 = r2-r1;

    f  = uinc/abs(d1);
    ds = f*uinc;
    p(np) = single(angle(d1));
    r1 = r1 + uinc*exp(i*double(p(np)));
    s1 = s1 + ds;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 7

function [data, err] = ppxy2p7(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with adapted s interval
% local prediction with last s interval
% position update by effective single angle
% does slightly improve err compared to ppxy2p6

r0 = ppval(ppxy, s0);
np = 0;
ds = uinc;

r1 = r0;
s1 = s0;
while s1 < se
    np = np + 1;

    r2 = ppval(ppxy, s1+ds);
    d1 = r2-r1;

    f  = uinc/abs(d1);
    ds = f*ds;
    p(np) = single(angle(d1));
    r1 = r1 + uinc*exp(i*double(p(np)));
    s1 = s1 + ds;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end

%% meth = 8

function [data, err] = ppxy2p8(ppxy, uinc)

[breaks,coefs,l,k,d] = unmkpp(ppxy); %#ok<NASGU>

s0 = breaks(1);
se = breaks(end);

% pre-allocate p with assumed size

np = ceil((se-s0)/uinc);
p = zeros(1, np, 'single');

% run over spline with adapted s interval
% local prediction with last s interval and iteration
% position update by effective single angle
% err is similar or slightly worse than ppxy2p7 for small uinc or
% slightly better for large uinc values.

r0 = ppval(ppxy, s0);
np = 0;
ds = uinc;

r1 = r0;
s1 = s0;
while s1 < se
    np = np + 1;
    for j = 1:10  % assume convergence after 10 iterations
        r2 = ppval(ppxy, s1+ds);
        d1 = r2-r1;

        f  = uinc/abs(d1);
        ds = f*ds;
        if abs(f-1) < 1e-10, break, end
    end
    p(np) = single(angle(d1));
    r1 = r1 + uinc*exp(i*double(p(np)));
    s1 = s1 + ds;
end

p = p(1:np);

% calculate integration error for last point

r1 = ppval(ppxy, s1); % by spline evaluation at last position s1
r2 = ppval(ppxy, s0) + sum(uinc.*exp(i*double(p))); % by phi integration
err = abs(r2-r1);

% save CRG data

data.head.ubeg = s0;
data.head.uend = s0 + length(p)*uinc;
data.head.uinc = uinc;

data.head.pbeg = p(1);
data.head.pend = p(end);

data.head.xbeg = real(r0);
data.head.ybeg = imag(r0);
data.head.xend = real(r1);
data.head.yend = imag(r1);

data.u = single([data.head.ubeg data.head.uend]);
data.p = p;

end
