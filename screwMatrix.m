function screwMat = screwMatrix(vec)

    screwMat = [    0,          -vec(3,1)   vec(2,1);
                    vec(3,1),   0,          -vec(1,1);
                    -vec(2,1),  vec(1,1),   0
               ];

end