#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1)
    { //�ݹ��������
// Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    { //objectsΪ2ʱ�����Ҹ���һ��
        node->left = recursiveBuild(std::vector{ objects[0] });
        node->right = recursiveBuild(std::vector{ objects[1] });

        node->bounds = Union(node->left->bounds, node->right->bounds);//����pMin pMax���ɴ��Χ��
        return node;
    }
    else
    {
        //
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
            Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (SplitMethod::SAH)
        {

        case SplitMethod::NAIVE: {

            switch (dim)
            {
            case 0: //��ʾ������x��
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                    {
                        return f1->getBounds().Centroid().x <
                            f2->getBounds().Centroid().x;
                    });//lambda���ʽ������sort���򷽷�����������Ա�������Χ�����ĵ�x���꣬С������ߣ�������ұ�
                break;
            case 1: //�����y��
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                    {
                        return f1->getBounds().Centroid().y <
                            f2->getBounds().Centroid().y;
                    });
                break;
            case 2: //�����z��
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                    {
                        return f1->getBounds().Centroid().z <
                            f2->getBounds().Centroid().z;
                    });
                break;
            }

            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling); //�����з�
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));
            node->left = recursiveBuild(leftshapes); //���ҿ�ʼ�ݹ�
            node->right = recursiveBuild(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);//����pMin pMax���ɴ��Χ��
        }
                               break;
        case SplitMethod::SAH:
        {
            float SN = centroidBounds.SurfaceArea();
            int B = 10;
            int mincostIndex = 0;
            float minCost = std::numeric_limits<float>::infinity(); //��С����

            switch (dim) //ֱ���ڿ�������������Ͽ�ʼ���ǵ��㷨
            {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                    {
                        return f1->getBounds().Centroid().x <
                            f2->getBounds().Centroid().x;
                    });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                    {
                        return f1->getBounds().Centroid().y <
                            f2->getBounds().Centroid().y;
                    });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                    {
                        return f1->getBounds().Centroid().z <
                            f2->getBounds().Centroid().z;
                    });
                break;
            }
            for (int i = 1; i < B; i++)
            {
                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() * i / B);
                auto ending = objects.end();
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);
                //�����Ұ�Χ��:
                Bounds3 leftBounds, rightBounds;
                for (int k = 0; k < leftshapes.size(); ++k)
                    leftBounds = Union(leftBounds, leftshapes[k]->getBounds().Centroid());
                for (int k = 0; k < rightshapes.size(); ++k)
                    rightBounds = Union(rightBounds, rightshapes[k]->getBounds().Centroid());
                float SA = leftBounds.SurfaceArea(); //SA
                float SB = rightBounds.SurfaceArea(); //SB
                float cost = 0.125 + (leftshapes.size() * SA + rightshapes.size() * SB) / SN; //���㻨��
                if (cost < minCost) //������Ѹ�С����¼��ǰ����ֵ
                {
                    minCost = cost;
                    mincostIndex = i;
                }
            }
            //�ҵ�mincostIndex��Ĳ�����ͬ��BVH
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() * mincostIndex / B);//���ֵ�ѡΪ��ǰ����Ͱ��λ��
            auto ending = objects.end();
            auto leftshapes = std::vector<Object*>(beginning, middling); //�����з�
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes); //���ҿ�ʼ�ݹ�
            node->right = recursiveBuild(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);//����pMin pMax���ɴ��Χ��
        }
        }
    }
    return node;
}




Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection intersection;

    if (node == nullptr || !node->bounds.IntersectP(ray, ray.direction_inv, {0,0,0}))
        return intersection;
    if (node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray); //������õ���obj���ࣺTriangle��getIntersection������Ҷ�ӽڵ�İ�Χ�п�ʼ����������
    }
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);
    
    return  hit1.distance < hit2.distance ? hit1 : hit2;
}
