#!/usr/bin/perl
use warnings;
use strict;

my $skip = <STDIN>;

my $file_count  = 0;
open(my $fh, '>', 'encoder_calib.dat');

while (<STDIN>) {
    if ($_ =~ 'recording') {
        close $fh;
        $file_count++;
        my $filename = 'encoder_' . $file_count . '.dat';
        open($fh, '>', $filename);
    }
    else {
        print $fh $_;
    }
}
close $fh;
