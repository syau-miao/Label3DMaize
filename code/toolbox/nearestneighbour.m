function [idx, tri] = nearestneighbour(varargin)
%NEARESTNEIGHBOUR    find nearest neighbours
%   IDX = NEARESTNEIGHBOUR(P, X) finds the nearest neighbour by Euclidean
%   distance to each point in P from X. P and X are both matrices with the
%   same number of rows, and points are the columns of the matrices. Output
%   is a vector of indices into X such that X(:, IDX) are the nearest
%   neighbours to P
%
%   IDX = NEARESTNEIGHBOUR(I, X) where I is a logical vector or vector of
%   indices, and X has at least two rows, finds the nearest neighbour in X
%   to each of the points X(:, I).
%   I must be a row vector to distinguish it from a single point.
%   If X has only one row, the first input is treated as a set of 1D points
%   rather than a vector of indices
%
%   IDX = NEARESTNEIGHBOUR(..., Property, Value)
%   Calls NEARESTNEIGHBOUR with the indicated parameters set.
%      Property:                         Value:
%      ---------                         ------  
%         NumberOfNeighbours             natural number, default 1
%            NEARESTNEIGHBOUR(..., 'NumberOfNeighbours', K) finds the closest
%            K points in ascending order to each point, rather than the
%            closest point
%
%         DelaunayMode                   {'on', 'off', |'auto'|}
%            DelaunayMode being set to 'on' means NEARESTNEIGHBOUR uses the
%            a Delaunay triangulation with dsearchn to find the points, if
%            possible. Setting it to 'auto' means NEARESTNEIGHBOUR decides
%            whether to use the triangulation, based on efficiency.
%
%         Triangulation                  Valid triangulation produced by
%                                        delaunay or delaunayn
%            If NEARESTNEIGHBOUR uses the Delaunay method, the
%            triangulation can be supplied, for a speed increase. Note that
%            DelaunayMode being 'auto' doesn't at this stage compensate for
%            a supplied triangulation, so it might pay to force
%            DelaunayMode to be 'on'.
%
%   [IDX, TRI] = NEARESTNEIGHBOUR( ... )
%   If the Delaunay Triangulation is used, TRI is the triangulation of X'.
%   Otherwise, TRI is an empty matrix
%
%   Example:
%     % Find the nearest neighbours to each point in p
%     p = rand(2, 5);
%     x = rand(2, 20);
%     idx = nearestneighbour(p, x)
%
%     % Find the five nearest neighbours to points x(:, [1 6 20]) in x
%     x = rand(4, 1000)
%     idx = nearestneighbour([1 6 20], x, 'NumberOfNeighbours', 5)
%
%   See also DELAUNAYN, DSEARCHN, TSEARCH

%TODO    Allow other metrics than Euclidean distance
%TODO    Implement the Delaunay mode for multiple neighbours
%TODO    Enhance the delaunaytest subfunction to allow for the
%        triangulation being supplied

% Copyright 2006 Richard Brown. This code may be freely used and
% distributed, so long as it maintains this copyright line
tri = [];  % Default value

% Default parameters
userParams.NumberOfNeighbours = 1     ;
userParams.DelaunayMode       = 'auto'; % {'on', 'off', |'auto'|}
userParams.Triangulation      = []    ;

% Parse inputs
[P, X, fIndexed, userParams] = parseinputs(userParams, varargin{:});

% Special case uses Delaunay triangulation for speed. 

% Determine whether to use Delaunay - set fDelaunay true or false
nX  = size(X, 2);
nP  = size(P, 2);
dim = size(X, 1);

switch lower(userParams.DelaunayMode)
  case 'on'
	%TODO Delaunay can't currently be used for finding more than one
	%neighbour
	fDelaunay = userParams.NumberOfNeighbours == 1 && ...
	  size(X, 2) > size(X, 1)                      && ...
	  ~fIndexed;
  case 'off'
	fDelaunay = false;
  case 'auto'
	fDelaunay = userParams.NumberOfNeighbours == 1 && ...
	  ~fIndexed                                    && ...
	  size(X, 2) > size(X, 1)                      && ...
	  delaunaytest(nX, nP, dim);
end

% Try doing Delaunay, if fDelaunay.
fDone = false;
if fDelaunay
  tri = userParams.Triangulation;
  if isempty(tri)
	try
	  tri   = delaunayn(X');
	catch
	  msgId = 'NearestNeighbour:DelaunayFail';
	  msg = ['Unable to compute delaunay triangulation, not using it. ',...
		'Set the DelaunayMode parameter to ''off'''];
	  warning(msgId, msg);
	end
  end
  if ~isempty(tri)
	try
	  idx = dsearchn(X', tri, P')';
	  fDone = true;
	catch
	  warning('NearestNeighbour:DSearchFail', ...
		'dsearchn failed on triangulation, not using Delaunay');
	end
  end
end

% If it didn't use Delaunay triangulation, find the neighbours directly by
% finding minimum distances
if ~fDone
  idx = zeros(userParams.NumberOfNeighbours, size(P, 2));

  % Loop through the set of points P, finding the neighbours
  Y = zeros(size(X));
  for iPoint = 1:size(P, 2)
	x = P(:, iPoint);

	% This is the faster than using repmat based techniques such as
	% Y = X - repmat(x, 1, size(X, 2))
	for i = 1:size(Y, 1)
	  Y(i, :) = X(i, :) - x(i);
	end

	% Find the closest points
	dSq = sum(abs(Y).^2, 1);
	if ~fIndexed
	  iSorted = minn(dSq, userParams.NumberOfNeighbours);
	else
	  iSorted = minn(dSq, userParams.NumberOfNeighbours + 1);
	  iSorted = iSorted(2:end);
	end
	idx(:, iPoint) = iSorted';
  end
end % if ~fDone

end % nearestneighbour




%DELAUNAYTEST   Work out whether the combination of dimensions makes
%fastest to use a Delaunay triangulation in conjunction with dsearchn.
%These parameters have been determined empirically on a Pentium M 1.6G /
%WinXP / 512MB / Matlab R14SP3 platform. Their precision is not
%particularly important
function tf = delaunaytest(nx, np, dim)
switch dim
  case 2
	tf = np > min(1.5 * nx, 400);
  case 3
	tf = np > min(4 * nx  , 1200);
  case 4
	tf = np > min(40 * nx , 5000);
	
  % if the dimension is higher than 4, it is almost invariably better not
  % to try to use the Delaunay triangulation
  otherwise 
	tf = false;
end % switch
end % delaunaytest




%MINN   find the n most negative elements in x, and return their indices
%  in ascending order
function I = minn(x, n)
%Note: preallocation with I = zeros(1,n) can actually slow the code down,
%particularly if the matrix is small. I've put it in, however, because it
%is good practice
%Feel free to comment the next line if you want
I = zeros(1, n);

% Sort by finding the minimum entry, storing it, and replacing with Inf
for i = 1:n
  [xmin, I(i)] = min(x);
  x(I(i)) = Inf;
end

end % minn




%PARSEINPUTS    Support function for nearestneighbour
function [P, X, fIndexed, userParams] = parseinputs(userParams, varargin)
P = varargin{1};
X = varargin{2};
varargin(1:2) = [];
% Check the dimensions of X and P
if size(X, 1) ~= 1
  % Check to see whether P is in fact a vector of indices
  if size(P, 1) == 1
	try
	  P = X(:, P);
	catch
	  error('NearestNeighbour:InvalidIndexVector', ...
		'Unable to index matrix using index vector');
	end
	fIndexed = true;
  else
	fIndexed = false;
  end % if size(P, 1) == 1
else % if size(X, 1) ~= 1
  fIndexed = false;
end

if ~fIndexed
  if size(P, 1) ~= size(X, 1)
	error('NearestNeighbour:DimensionMismatch', ...
	  'No. of rows of input arrays doesn''t match');
  end
end

% Parse the Property/Value pairs
if rem(length(varargin), 2) ~= 0
  error('NearestNeighbour:propertyValueNotPair', ...
	'Additional arguments must take the form of Property/Value pairs');
end

while length(varargin) ~= 0
  property = varargin{1};
  value    = varargin{2};

  switch lower(property)
	case 'numberofneighbours'
	  if rem(value, 1) ~= 0 || ...
		  value > length(X) - double(fIndexed) || ...
		  value < 1
		error('NearestNeighbour:InvalidNumberOfNeighbours', ...
		  'Number of Neighbours must be an integer, and smaller than the no. of points in X');
	  end
	  userParams.NumberOfNeighbours = value;

	case 'delaunaymode'
	  fOn = strcmpi(value, 'on');
	  if strcmpi(value, 'off')
		userParams.DelaunayMode = 'off';
	  elseif fOn || strcmpi(value, 'auto')
		if userParams.NumberOfNeighbours ~= 1
		  if fOn
			warning('NearestNeighbour:TooMuchForDelaunay', ...
			  'Delaunay Triangulation method works only for one neighbour');
		  end
		  userParams.DelaunayMode = 'off';
		elseif size(X, 2) < size(X, 1) + 1
		  if fOn
			warning('NearestNeighbour:TooFewDelaunayPoints', ...
			  'Insufficient points to compute Delaunay triangulation');
		  end
		  userParams.DelaunayMode = 'off';
			
		elseif size(X, 1) == 1
		  if fOn
			warning('NearestNeighbour:DelaunayDimensionOne', ...
			  'Cannot compute Delaunay triangulation for 1D input');
		  end
		  userParams.DelaunayMode = 'off';
		else
		  userParams.DelaunayMode = value;
		end
	  else
		warning('NearestNeighbour:InvalidOption', ...
		  'Invalid Option');
	  end % if strcmpi(value, 'off')
	  
	  
	case 'triangulation'
	  if isnumeric(value) && size(value, 2) == size(X, 1) + 1 && ...
		  all(ismember(1:size(X, 2), value))
		userParams.Triangulation = value;
	  else
		error('NearestNeighbour:InvalidTriangulation', ...
		  'Triangulation not a valid Delaunay Triangulation');
	  end
	otherwise
	  error('NearestNeighbour:InvalidProperty', 'Invalid Property');
  end % switch lower(property)

  varargin(1:2) = [];
end % while

end %parseinputs