function [D,L,u,v]=sinkhornTransport(a,b,K,U,lambda,stoppingCriterion,p_norm,tolerance,maxIter,VERBOSE)
% Compute N dual-Sinkhorn divergences (upper bound on the EMD) as well as
% N lower bounds on the EMD for all the pairs
%
% D= [d(a_1,b_1), d(a_2,b_2), ... , d(a_N,b_N)].
% If needed, the function also outputs diagonal scalings to recover smoothed optimal
% transport between each of the pairs (a_i,b_i).
%
%---------------------------
% Required Inputs:
%---------------------------
% a is either
%    - a d1 x 1 column vector in the probability simplex (nonnegative,
%    summing to one). This is the [1-vs-N mode]
%    - a d_1 x N matrix, where each column vector is in the probability simplex
%      This is the [N x 1-vs-1 mode]
%
% b is a d2 x N matrix of N vectors in the probability simplex
%
% K is a d1 x d2 matrix, equal to exp(-lambda M), where M is the d1 x d2
% matrix of pairwise distances between bins described in a and bins in the b_1,...b_N histograms.
% In the most simple case d_1=d_2 and M is simply a distance matrix (zero
% on the diagonal and such that m_ij < m_ik + m_kj
%
%
% U = K.*M is a d1 x d2 matrix, pre-stored to speed up the computation of
% the distances.
%
%
%---------------------------
% Optional Inputs:
%---------------------------
% stoppingCriterion in {'marginalDifference','distanceRelativeDecrease'}
%   - marginalDifference (Default) : checks whether the difference between
%              the marginals of the current optimal transport and the
%              theoretical marginals set by a b_1,...,b_N are satisfied.
%   - distanceRelativeDecrease : only focus on convergence of the vector
%              of distances
%
% p_norm: parameter in {(1,+infty]} used to compute a stoppingCriterion statistic
% from N numbers (these N numbers might be the 1-norm of marginal
% differences or the vector of distances.
%
% tolerance : >0 number to test the stoppingCriterion.
%
% maxIter: maximal number of Sinkhorn fixed point iterations.
% 
% verbose: verbose level. 0 by default.
%---------------------------
% Output
%---------------------------
% D : vector of N dual-sinkhorn divergences, or upper bounds to the EMD.
%
% L : vector of N lower bounds to the original OT problem, a.k.a EMD. This is computed by using
% the dual variables of the smoothed problem, which, when modified
% adequately, are feasible for the original (non-smoothed) OT dual problem
%
% u : d1 x N matrix of left scalings
% v : d2 x N matrix of right scalings
%
% The smoothed optimal transport between (a_i,b_i) can be recovered as
% T_i = diag(u(:,i)) * K * diag(v(:,i));
%
% or, equivalently and substantially faster:
% T_i = bsxfun(@times,v(:,i)',(bsxfun(@times,u(:,i),K)))
%
%
% Relevant paper:
% M. Cuturi,
% Sinkhorn Distances : Lightspeed Computation of Optimal Transport,
% Advances in Neural Information Processing Systems (NIPS) 26, 2013

% This code, (c) Marco Cuturi 2013,2014 (see license block below)
% v0.2b corrected a small bug in the definition of the first scaling
% variable u.
% v0.2 numerous improvements, including possibility to compute
%      simultaneously distances between different pairs of points 24/03/14
% v0.1 added lower bound 26/11/13
% v0.0 first version 20/11/2013

% Change log:
% 28/5/14: The initialization of u was u=ones(length(a),size(b,2))/length(a); which does not
%          work when the number of columns of a is larger than the number
%          of lines (i.e. more histograms than dimensions). The correct
%          initialization must use size(a,1) and not its length.
% 24/3/14: Now possible to compute in parallel D(a_i,b_i) instead of being
% limited to D(a,b_i). More optional inputs and better error checking.
% Removed an unfortunate code error where 2 variables had the same name.
%
% 20/1/14: Another correction at the very end of the script to output weights.
%
% 15/1/14: Correction when outputting l at the very end of the script. replaced size(b) by size(a).

%% Processing optional inputs

if nargin<6 || isempty(stoppingCriterion),
    stoppingCriterion='marginalDifference'; % check marginal constraints by default
end

if nargin<7 || isempty(p_norm),
    p_norm=inf;
end

if nargin<8 || isempty(tolerance),
    tolerance=.5e-2;
end

if nargin<9 || isempty(maxIter),
    maxIter=5000;
end

if nargin<10 || isempty(VERBOSE),
    VERBOSE=0;
end


%% Checking the type of computation: 1-vs-N points or many pairs.

if size(a,2)==1,
    ONE_VS_N=true; % We are computing [D(a,b_1), ... , D(a,b_N)]
elseif size(a,2)==size(b,2),
    ONE_VS_N=false; % We are computing [D(a_1,b_1), ... , D(a_N,b_N)]
else
    error('The first parameter a is either a column vector in the probability simplex, or N column vectors in the probability simplex where N is size(b,2)');
end

%% Checking dimensionality:
if size(b,2)>size(b,1),
    BIGN=true;
else
    BIGN=false;
end


%% Small changes in the 1-vs-N case to go a bit faster.
if ONE_VS_N, % if computing 1-vs-N make sure all components of a are >0. Otherwise we can get rid of some lines of K to go faster.
    I=(a>0);
    someZeroValues=false;
    if ~all(I), % need to update some vectors and matrices if a does not have full support
        someZeroValues=true;
        K=K(I,:);
        U=U(I,:);
        a=a(I);
    end
    ainvK=bsxfun(@rdivide,K,a); % precomputation of this matrix saves a d1 x N Schur product at each iteration.
end

%% Fixed point counter
compt=0;

%% Initialization of Left scaling Factors, N column vectors.
u=ones(size(a,1),size(b,2))/size(a,1);


if strcmp(stoppingCriterion,'distanceRelativeDecrease')
    Dold=ones(1,size(b,2)); %initialization of vector of distances.
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Fixed Point Loop
% The computation below is mostly captured by the repeated iteration of
% u=a./(K*(b./(K'*u)));
%
% In some cases, this iteration can be sped up further when considering a few
% minor tricks (when computing the distances of 1 histogram vs many,
% ONE_VS_N, or when the number of histograms N is larger than the dimension
% of these histograms). 
% We consider such cases below.


while compt<maxIter,
    if ONE_VS_N, % 1-vs-N mode
        if BIGN
            u=1./(ainvK*(b./(K'*u))); % main iteration of Sinkhorn's algorithm
        else
            u=1./(ainvK*(b./(u'*K)'));
        end
    else % N times 1-vs-1 mode
        if BIGN
            u=a./(K*(b./(u'*K)'));
        else
            u=a./(K*(b./(K'*u)));
        end
    end
    compt=compt+1;
    
    % check the stopping criterion every 20 fixed point iterations
    % or, if that's the case, before the final iteration to store the most
    % recent value for the matrix of right scaling factors v.
    if mod(compt,20)==1 || compt==maxIter,   
        % split computations to recover right and left scalings.        
        if BIGN
            v=b./(K'*u); % main iteration of Sinkhorn's algorithm
        else
            v=b./((u'*K)');
        end
        
        if ONE_VS_N, % 1-vs-N mode
            u=1./(ainvK*v);
        else
            u=a./(K*v);            
        end
                        
        % check stopping criterion
        switch stoppingCriterion,
            case 'distanceRelativeDecrease',
                D=sum(u.*(U*v));
                Criterion=norm(D./Dold-1,p_norm);
                if Criterion<tolerance || isnan(Criterion),
                    break;
                end
                Dold=D;
                
            case 'marginalDifference',
                Criterion=norm(sum(abs(v.*(K'*u)-b)),p_norm);
                if Criterion<tolerance || isnan(Criterion), % norm of all || . ||_1 differences between the marginal of the current solution with the actual marginals.
                    break;
                end
            otherwise
                error('Stopping Criterion not recognized');        
        end        
        compt=compt+1;
        if VERBOSE>0,
            disp(['Iteration :',num2str(compt),' Criterion: ',num2str(Criterion)]);        
        end
        if any(isnan(Criterion)), % stop all computation if a computation of one of the pairs goes wrong.
            error('NaN values have appeared during the fixed point iteration. This problem appears because of insufficient machine precision when processing computations with a regularization value of lambda that is too high. Try again with a reduced regularization parameter lambda or with a thresholded metric matrix M.');
        end
    end
end

if strcmp(stoppingCriterion,'marginalDifference'), % if we have been watching marginal differences, we need to compute the vector of distances.
    D=sum(u.*(U*v));
end

if nargout>1, % user wants lower bounds
    alpha = log(u);
    beta = log(v);
    beta(beta==-inf)=0; % zero values of v (corresponding to zero values in b) generate inf numbers.
    if ONE_VS_N
        L= (a'* alpha + sum(b.*beta))/lambda;
    else        
        alpha(alpha==-inf)=0; % zero values of u (corresponding to zero values in a) generate inf numbers. in ONE-VS-ONE mode this never happens.
        L= (sum(a.*alpha) + sum(b.*beta))/lambda;
    end
    
    
end

if nargout>2 && ONE_VS_N && someZeroValues, % user wants scalings. We might have to arficially add zeros again in bins of a that were suppressed.
    uu=u;
    u=zeros(length(I),size(b,2));
    u(I,:)=uu;
end



% ***** BEGIN LICENSE BLOCK *****
%  * Version: MPL 1.1/GPL 2.0/LGPL 2.1
%  *
%  * The contents of this file are subject to the Mozilla Public License Version
%  * 1.1 (the "License"); you may not use this file except in compliance with
%  * the License. You may obtain a copy of the License at
%  * http://www.mozilla.org/MPL/
%  *
%  * Software distributed under the License is distributed on an "AS IS" basis,
%  * WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
%  * for the specific language governing rights and limitations under the
%  * License.
%  *
%  * The Original Code is Sinkhorn Transport, (C) 2013, Marco Cuturi
%  *
%  * The Initial Developers of the Original Code is
%  *
%  * Marco Cuturi   mcuturi@i.kyoto-u.ac.jp
%  *
%  * Portions created by the Initial Developers are
%  * Copyright (C) 2013 the Initial Developers. All Rights Reserved.
%  *
%  *
%  ***** END LICENSE BLOCK *****
