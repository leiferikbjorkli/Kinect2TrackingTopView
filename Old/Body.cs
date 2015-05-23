using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InteractionDetection
{
    public class Body
    {
        private int id;
        private int[] headCoG;

        public Body(int id, int[] headCoG)
        {
            this.id = id;
            this.headCoG = headCoG;
            
        }


        public int[] HeadCoG
        {
            get { return headCoG; }
            set { headCoG = value; }
        }
            

    }
}
