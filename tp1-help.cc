// Example of world building, display, and successor computation for the artificial 
// intelligence path-finding lab
//
// Author: Didier Lime
// Date: 2018-10-03

#include <iostream>
#include <list>
#include <vector>
#include <cstdlib>
#include <ctime>

using namespace std;

class World
{
    private:
        // Number of columns
        unsigned int L;
        
        // Number of lines
        unsigned int H;

        // Unidimensional array for tiles
        int* w;
    
    public:
        // Constructor
        World(unsigned int L, unsigned int H, double P)
        {
            this->L = L;
            this->H = H;
            this->w = new int[L*H]();

            // Add walls to the first and last columns
            for (unsigned int i = 0; i < H; i++)
            {
                this->w[i * L] = 1;
                this->w[i * L + L - 1] = 1;
            }

            // Add walls to the first and last lines
            for (unsigned int j = 0; j < L; j++)
            {
                this->w[j] = 1;
                this->w[(H - 1) * L + j] = 1;
            }

            for (unsigned int i = 0; i < H; i++)
            {
                for (unsigned int j = 0; j < L; j++)
                {
                    // add a wall in this tile with probability P and provided that it is neither
                    // the starting tile nor the goal tile 
                    if ((double) rand() / RAND_MAX < P && !(i == 1 && j == 1) && !(i == H - 2 && j == L - 2))
                    {
                        this->w[i * L + j] = 1;
                    }
                }
            }
        }

        // Display the world
        void display()
        {
            for (unsigned int i = 0; i < H; i++)
            {
                for (unsigned int j = 0; j < L; j++)
                {
                    switch (this->w[i * L + j])
                    {
                        case 0:
                            cout << ".";
                            break;

                        case 1:
                            cout << "W";
                            break;
                    }
                }
                cout << endl;
            }
        }

        // compute the successors of tile number i in world w
        // we return the number n of valid successors
        // the actual list is in array r where only the first n
        // elements are significant
        vector<unsigned int> successors(unsigned int i)
        {
            vector<unsigned int> R;

            if (i >= 0 && i < this->L * this->H && this->w[i] != 1)
            {
                // if i is a correct tile number (inside the array and not on a wall)
                // look in the four adjacent tiles and keep only those with no wall
                const unsigned int moves[] = { i - 1, i + 1, i - L, i + L};
                
                for (unsigned int k = 0; k < 4; k++)
                {
                    if (this->w[moves[k]] != 1)
                    {
                        R.push_back(moves[k]);
                    }
                }
            }

            return R;
        }

        // Destructor
        ~World()
        {
            delete [] this->w;
        }

        // Depth-first search
        // starting from tile number s0, find a path to tile number t
        // return true if such a path exists, false otherwise
        // if it exists the path is given in variable path (hence the reference &)
        bool dfs(unsigned int s0, unsigned int t, list<unsigned int>& path)
        {
            bool r = false;

            // ... Complete here ...

            return r;
        } 
};

int main()
{
    // Initialise the random number generator
    srand(time(0));

    // Create a world
    World w(20, 10, 0.2);

    // Display it
    w.display();

    // Print the tile numbers of the successors of the starting tile (1, 1)
    unsigned int succs[4];

    for (auto succ: w.successors(21))
    {
        cout << succ << " ";
    }
    cout << endl;

    return 0;
}


