## Copyright (C) 2019 Munzir Zafar
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} plot_results (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Munzir Zafar <munzir@vision>
## Created: 2019-02-18

adrc_output = dlmread('output_with_adrc.csv', ' ');

adrc_output_ = zeros(length(adrc_output), 5); 
for i=1:length(adrc_output)
  cnt= 0; 
  for j=1:size(adrc_output,2)
    if(adrc_output(i, j))
      cnt = cnt + 1; 
      adrc_output_(i, cnt) = adrc_output(i, j); 
    end
  end
end

lqr_output = dlmread('output_without_adrc.csv', ' ');
lqt_output_ = zeros(length(lqr_output), 5); 
for i=1:length(lqr_output)
  cnt= 0; 
  for j=1:size(lqr_output,2)
    if(lqr_output(i, j))
      cnt = cnt + 1; 
      lqr_output_(i, cnt) = lqr_output(i, j); 
    end
  end
end

t1 = 0.001*(1:length(adrc_output_));
t2 = 0.001*(1:length(lqr_output));
plot(adrc_output_(:,3), adrc_output_(:,4), lqr_output_(:,3), lqr_output_(:,4));
legend({'with adrc', 'without adrc'});