#!/usr/bin/perl -W
use strict;
use Cwd;

my $dir = getcwd;

print "\ncleaning kernel source\n";
system ("make mrproper");

print "\nremoving old boot.img\n";
system ("rm boot.img");
system ("rm $dir/zpack/zcwmfiles/boot.img");

print "\nremoving old n860_tst_krnl.zip\n";
system ("rm $dir/n860_tst_krnl.zip");
