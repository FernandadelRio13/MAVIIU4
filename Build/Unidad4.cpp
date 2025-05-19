#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>
#include <vector>
#include <cmath>
#include <set>

const float SCALE = 30.f;
const int WIDTH = 800;
const int HEIGHT = 600;

b2World world(b2Vec2(0.f, 9.8f));

sf::Vector2f b2ToSf(const b2Vec2& v) {
    return sf::Vector2f(v.x * SCALE, v.y * SCALE);
}

b2Vec2 sfToB2(const sf::Vector2f& v) {
    return b2Vec2(v.x / SCALE, v.y / SCALE);
}

struct Ragdoll {
    std::vector<b2Body*> parts;
    std::vector<b2Joint*> joints;
};

Ragdoll createRagdoll(b2Vec2 position, b2Vec2 initialVelocity) {
    Ragdoll ragdoll;

    float torsoW = 0.6f, torsoH = 1.2f;
    float limbW = 0.3f, limbH = 0.6f;
    float headR = 0.35f;

    b2BodyDef torsoDef;
    torsoDef.type = b2_dynamicBody;
    torsoDef.position = position;
    b2Body* torso = world.CreateBody(&torsoDef);
    b2PolygonShape torsoShape;
    torsoShape.SetAsBox(torsoW / 2, torsoH / 2);
    torso->CreateFixture(&torsoShape, 1.0f);
    torso->SetLinearVelocity(initialVelocity);
    ragdoll.parts.push_back(torso);

    b2BodyDef headDef = torsoDef;
    headDef.position = position + b2Vec2(0, -torsoH / 2 - headR);
    b2Body* head = world.CreateBody(&headDef);
    b2CircleShape headShape;
    headShape.m_radius = headR;
    head->CreateFixture(&headShape, 1.0f);
    head->SetLinearVelocity(initialVelocity);
    ragdoll.parts.push_back(head);

    b2RevoluteJointDef neckJoint;
    neckJoint.Initialize(torso, head, torso->GetWorldCenter() + b2Vec2(0, -torsoH / 2));
    ragdoll.joints.push_back(world.CreateJoint(&neckJoint));

    for (int i = -1; i <= 1; i += 2) {
        b2BodyDef armDef = torsoDef;
        armDef.position = torso->GetWorldCenter() + b2Vec2(i * (torsoW / 2 + limbW / 2), -torsoH / 4);
        b2Body* arm = world.CreateBody(&armDef);
        b2PolygonShape armShape;
        armShape.SetAsBox(limbW / 2, limbH / 2);
        arm->CreateFixture(&armShape, 1.0f);
        arm->SetLinearVelocity(initialVelocity);
        ragdoll.parts.push_back(arm);

        b2RevoluteJointDef shoulder;
        shoulder.Initialize(torso, arm, torso->GetWorldCenter() + b2Vec2(i * torsoW / 2, -torsoH / 4));
        ragdoll.joints.push_back(world.CreateJoint(&shoulder));
    }

    for (int i = -1; i <= 1; i += 2) {
        b2BodyDef legDef = torsoDef;
        legDef.position = torso->GetWorldCenter() + b2Vec2(i * torsoW / 4, torsoH / 2 + limbH / 2);
        b2Body* leg = world.CreateBody(&legDef);
        b2PolygonShape legShape;
        legShape.SetAsBox(limbW / 2, limbH / 2);
        leg->CreateFixture(&legShape, 1.0f);
        leg->SetLinearVelocity(initialVelocity);
        ragdoll.parts.push_back(leg);

        b2RevoluteJointDef hip;
        hip.Initialize(torso, leg, torso->GetWorldCenter() + b2Vec2(i * torsoW / 4, torsoH / 2));
        ragdoll.joints.push_back(world.CreateJoint(&hip));
    }

    return ragdoll;
}

b2Body* createBox(b2Vec2 position, b2Vec2 size, bool isDynamic) {
    b2BodyDef bodyDef;
    bodyDef.position = position;
    bodyDef.type = isDynamic ? b2_dynamicBody : b2_staticBody;
    b2Body* body = world.CreateBody(&bodyDef);

    b2PolygonShape shape;
    shape.SetAsBox(size.x / 2, size.y / 2);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = 1.f;
    fixtureDef.friction = 0.5f;
    fixtureDef.restitution = 0.1f;

    body->CreateFixture(&fixtureDef);
    return body;
}

class MyContactListener : public b2ContactListener {
public:
    b2Body* boxBody;
    b2Body* circleBody;
    std::set<b2Body*> toActivate;

    MyContactListener(b2Body* box, b2Body* circle) : boxBody(box), circleBody(circle) {}

    void BeginContact(b2Contact* contact) override {
        b2Fixture* fixA = contact->GetFixtureA();
        b2Fixture* fixB = contact->GetFixtureB();

        b2Body* bodyA = fixA->GetBody();
        b2Body* bodyB = fixB->GetBody();

        if ((bodyA == boxBody || bodyA == circleBody) && bodyB->GetType() == b2_dynamicBody) {
            if (bodyA->GetType() == b2_staticBody) {
                toActivate.insert(bodyA);
            }
        }
        else if ((bodyB == boxBody || bodyB == circleBody) && bodyA->GetType() == b2_dynamicBody) {
            if (bodyB->GetType() == b2_staticBody) {
                toActivate.insert(bodyB);
            }
        }
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Ragdoll Cannon");
    window.setFramerateLimit(60);

    createBox(b2Vec2(WIDTH / 2 / SCALE, HEIGHT / SCALE), b2Vec2(WIDTH / SCALE, 1.f), false);
    createBox(b2Vec2(0, HEIGHT / 2 / SCALE), b2Vec2(1.f, HEIGHT / SCALE), false);
    createBox(b2Vec2(WIDTH / SCALE, HEIGHT / 2 / SCALE), b2Vec2(1.f, HEIGHT / SCALE), false);
    createBox(b2Vec2(WIDTH / 2 / SCALE, 0), b2Vec2(WIDTH / SCALE, 1.f), false);

    b2Body* boxBody = createBox(b2Vec2(13.f, 11.f), b2Vec2(3.f, 3.f), false);
    b2Body* wallBody = createBox(b2Vec2(16.f, 19.f), b2Vec2(2.f, 5.f), false);

    b2Body* circleBody = nullptr;
    {
        b2BodyDef circleDef;
        circleDef.position.Set(6.f, 5.f);
        circleDef.type = b2_staticBody;
        circleBody = world.CreateBody(&circleDef);

        b2CircleShape circleShape;
        circleShape.m_radius = 1.5f;

        b2FixtureDef circleFixture;
        circleFixture.shape = &circleShape;
        circleFixture.density = 1.f;
        circleFixture.friction = 0.5f;
        circleFixture.restitution = 0.1f;

        circleBody->CreateFixture(&circleFixture);
    }

    MyContactListener contactListener(boxBody, circleBody);
    world.SetContactListener(&contactListener);

    std::vector<Ragdoll> ragdolls;
    sf::Clock chargeClock;
    bool charging = false;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                charging = true;
                chargeClock.restart();
            }

            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
                charging = false;

                sf::Vector2f cannonPos(50, HEIGHT - 50);
                sf::Vector2f mousePos = sf::Vector2f(sf::Mouse::getPosition(window));
                sf::Vector2f dir = mousePos - cannonPos;

                float angle = atan2(dir.y, dir.x);
                float timeHeld = chargeClock.getElapsedTime().asSeconds();
                float base = 6.f;
                float maxPower = 20.f;
                float chargeRate = 14.f;
                float power = std::min(maxPower, base + timeHeld * chargeRate);

                b2Vec2 spawnPos = sfToB2(cannonPos);
                Ragdoll rag = createRagdoll(spawnPos, b2Vec2(std::cos(angle) * power, std::sin(angle) * power));
                ragdolls.push_back(rag);
            }
        }

        for (b2Body* body : contactListener.toActivate) {
            body->SetType(b2_dynamicBody);
            b2MassData massData;
            body->GetMassData(&massData);
            if (massData.mass == 0.f) {
                body->ResetMassData();
            }
        }
        contactListener.toActivate.clear();

        world.Step(1 / 60.f, 8, 3);
        window.clear(sf::Color::Black);

        sf::Vector2f cannonBase(50, HEIGHT - 50);
        sf::RectangleShape cannon(sf::Vector2f(60, 15));
        sf::Vector2f mouse = sf::Vector2f(sf::Mouse::getPosition(window));
        sf::Vector2f diff = mouse - cannonBase;
        float rotation = std::atan2(diff.y, diff.x) * 180 / 3.14159f;
        cannon.setRotation(rotation);
        cannon.setPosition(cannonBase);
        cannon.setFillColor(sf::Color::White);
        window.draw(cannon);

        for (b2Body* body = world.GetBodyList(); body; body = body->GetNext()) {
            for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
                if (f->GetType() == b2Shape::e_polygon) {
                    b2PolygonShape* s = (b2PolygonShape*)f->GetShape();
                    sf::ConvexShape poly;
                    poly.setPointCount(s->m_count);
                    for (int i = 0; i < s->m_count; i++) {
                        b2Vec2 p = body->GetWorldPoint(s->m_vertices[i]);
                        poly.setPoint(i, b2ToSf(p));
                    }
                    if (body == boxBody) {
                        poly.setFillColor(sf::Color::Yellow);
                    }
                    else if (body == wallBody) {
                        poly.setFillColor(sf::Color(150, 150, 150));
                    }
                    else {
                        poly.setFillColor(body->GetType() == b2_staticBody ? sf::Color::Green : sf::Color::Yellow);
                    }
                    window.draw(poly);
                }
                else if (f->GetType() == b2Shape::e_circle) {
                    b2CircleShape* s = (b2CircleShape*)f->GetShape();
                    b2Vec2 pos = body->GetPosition();
                    float r = s->m_radius * SCALE;
                    sf::CircleShape circle(r);
                    circle.setOrigin(r, r);
                    circle.setPosition(pos.x * SCALE, pos.y * SCALE);

                    if (body == circleBody) {
                        circle.setFillColor(sf::Color::Red);
                    }
                    else {
                        circle.setFillColor(sf::Color::Magenta);
                    }

                    window.draw(circle);
                }
            }
        }

        for (auto& rag : ragdolls) {
            for (b2Body* part : rag.parts) {
                for (b2Fixture* f = part->GetFixtureList(); f; f = f->GetNext()) {
                    if (f->GetType() == b2Shape::e_polygon) {
                        b2PolygonShape* s = (b2PolygonShape*)f->GetShape();
                        sf::ConvexShape poly;
                        poly.setPointCount(s->m_count);
                        for (int i = 0; i < s->m_count; i++) {
                            b2Vec2 p = part->GetWorldPoint(s->m_vertices[i]);
                            poly.setPoint(i, b2ToSf(p));
                        }
                        poly.setFillColor(sf::Color::Cyan);
                        window.draw(poly);
                    }
                    else if (f->GetType() == b2Shape::e_circle) {
                        b2CircleShape* s = (b2CircleShape*)f;
                        b2Vec2 pos = part->GetPosition();
                        float r = s->m_radius * SCALE;
                        sf::CircleShape circle(r);
                        circle.setOrigin(r, r);
                        circle.setPosition(pos.x * SCALE, pos.y * SCALE);
                        circle.setFillColor(sf::Color::Magenta);
                        window.draw(circle);
                    }
                }
            }
        }

        window.display();
    }

    return 0;
}