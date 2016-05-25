#!/usr/bin/perl
use warnings;
use strict;


my $file_count  = 0;
open(my $fh, '>', 'empty');

while (<STDIN>) {
    if ($_ =~ 'recording') {
        close $fh;
        my $filename = 'encoder_' . $file_count . '.dat';
        $file_count++;
        open($fh, '>', $filename);
    }
    else {
        print $fh $_;
    }
}
close $fh;
