Matrix2i a; a << 1, 2, 3, 4;
cout << "Here is the matrix a:\n" << a << endl;

a = a.transpose(); // !!! do NOT do this !!! why tho?
cout << "and the result of the aliasing effect:\n" << a << endl;