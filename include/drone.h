#ifndef LIBDRONE_DRONE_H
#define LIBDRONE_DRONE_H

#include <types.h>
#include <commands.h>
#include <dronevisitor.h>
#include <interface/iconnectionstatuslistener.h>
#include <interface/inavdatalistener.h>
#include <interface/istatuslistener.h>

#include <vector>
#include <atomic>
#include <memory>
#include <mutex>

#include <boost/thread.hpp>

class Drone
{
	public:
		virtual ~Drone() {};

		/*
		 * Visitor accept function
		 */

		virtual drone::error accept(dronevisitor *visitor) = 0;

		/*
		 * Setup related functions
		*/
		drone::connectionstatus connect();
		bool isConnected();

		void addNavdataListener(INavdataListener *listener);
		void removeNavdataListener(INavdataListener *listener);
		void addStatusListener(IStatusListener *listener);
		void removeStatusListener(IStatusListener *listener);
		void addConnectionStatusListener(IConnectionStatusListener *listener);
		void removeConnectionStatusListener(IConnectionStatusListener *listener);

		bool startUpdateLoop(); // Returns false if not connected, true otherwise
		void stopUpdateLoop();

		/*
		 * Navdata retrieval functions
		 */
		std::shared_ptr<const drone::navdata> getNavdata();
		virtual drone::limits getLimits() = 0; // Returns limits for in-flight angles and speeds
		virtual drone::config getConfig() = 0;

		bool isFlying();

		bool isArmed();

		/*
		 * Configuration functions
		 */

		virtual void setLimits(drone::limits limits) = 0;
		virtual void setConfig(drone::config config) = 0;

		/*
		 * Drone flight commands
		 */
		drone::error arm();
		drone::error disarm();
		drone::error addCommand(drone::command command); // Returns false if not connected, true otherwise

		/*
		 * Utility functions
		 */
        static float applyLimit(float value, float threshold); // Clamp value so it never exceeds -threshold or threshold
	protected:
		/*
		 * Event listener notification functions (to be used by drone implementation)
		 */
		void notifyNavdataListeners(std::shared_ptr<const drone::navdata> navdata);

		void notifyConnectionEstablished();
		void notifyConnectionLost();

		void markConnectionLost();

		/*
		 * Call this from your drone implementation
		 */
		void notifyStatusListeners(int status);

		/*
		 * To be implemented by specific drone implementation
		 */
		virtual drone::connectionstatus tryConnecting() = 0;
		virtual void beforeUpdate() = 0; // Gets run at the beginning of each update cycle
		virtual void updateCycle() = 0; // Gets run at a constant rate (at the end of the update cycle), put any miscellaneous functionality in here
		virtual bool decodeNavdata(std::shared_ptr<drone::navdata> &navdata) = 0; // Retrieve and process navdata here, store it in navdata given as parameter and return false if no new navdata available
		virtual bool processCommand(drone::command &command) = 0; // Process commands here, return false on failure
		virtual bool processNoCommand() = 0; // Gets run when there is no command in the command queue; return false on failure
		virtual void connectionLost() = 0; // Handle unexpected loss of connection here

	private:
		void runUpdateLoop();

		std::atomic<bool> _stop_flag{false};
		std::atomic<bool> _connected{false};
		std::atomic<bool> _armed{false};

		boost::thread *_updater = nullptr;

		std::mutex _commandmutex;
		std::vector<drone::command> _commandqueue;

		std::vector<INavdataListener *> _ndlisteners;
		std::vector<IStatusListener *> _slisteners;
		std::vector<IConnectionStatusListener *> _cslisteners;
};

#endif
