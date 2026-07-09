'use client';
import React, { useState, useEffect } from 'react';
import Image from 'next/image';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import {
  HomeIcon,
  DocumentReportIcon,
  MapIcon,
  AdjustmentsIcon,
  ShieldCheckIcon,
  MenuAlt2Icon,
  XIcon,
  UserCircleIcon,
} from '@heroicons/react/outline';
import LogoutButton from './LogoutButton';
import { fetchUsername } from '../utils/fetchUsername';

const isDemoMode = process.env.NEXT_PUBLIC_DEMO_MODE === 'true';

const navigation = [
  { name: 'Home', icon: HomeIcon, href: '/' },
  { name: 'Analysis', icon: DocumentReportIcon, href: '/reports' },
  { name: 'Map', icon: MapIcon, href: '/map' },
  { name: 'Manual Override', icon: AdjustmentsIcon, href: '/livecam' },
  { name: 'Security', icon: ShieldCheckIcon, href: '/override' },
];

const Sidebar: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [loaded, setLoaded] = useState(false);
  const [username, setUsername] = useState<string | null>(null);
  const pathname = usePathname();

  useEffect(() => {
    setLoaded(true);
    const getUsername = async () => {
      const fetchedUsername = await fetchUsername();
      setUsername(fetchedUsername);
    };
    getUsername();
  }, []);

  const isActive = (href: string) =>
    href === '/' ? pathname === '/' : pathname.startsWith(href);

  return (
    <>
      <div className="md:hidden flex items-center justify-between w-full bg-gray-50 text-black px-4 py-3 shadow-md">
        <button onClick={() => setIsOpen(!isOpen)} aria-label="Toggle Sidebar" className="focus:outline-none">
          {isOpen ? <XIcon className="h-6 w-6" /> : <MenuAlt2Icon className="h-6 w-6" />}
        </button>
        <div className="flex items-center">
          <Image src="/icons/logo_main.svg" width={34} height={34} alt="logo" />
          <span className="ml-2 font-bold font-stacker text-xl">Eco-Dex</span>
        </div>
        <div className="w-6" />
      </div>

      <aside
        className={`fixed inset-y-0 left-0 transform transition-transform duration-500 ease-in-out ${
          loaded ? 'translate-x-0' : '-translate-x-full'
        } ${isOpen ? 'md:translate-x-0' : '-translate-x-full'} md:translate-x-0 bg-gray-50 text-black w-52 z-50 border-r-2 shadow-lg`}
      >
        <div className="flex flex-col h-full p-6 border-b-1">
          <div className="flex items-center mb-8 border-b border-gray-300 pb-4">
            <Image src="/icons/logo_main.svg" width={34} height={34} alt="logo" />
            <span className="ml-2 text-lg font-stacker font-bold">Eco-Dex</span>
          </div>

          <nav className="flex-1">
            <ul className="space-y-1">
              {navigation.map((item) => (
                <li key={item.name}>
                  <Link
                    href={item.href}
                    onClick={() => setIsOpen(false)}
                    className={`flex items-center w-full px-4 py-3 text-medium font-medium transition-all duration-200 rounded-md ${
                      isActive(item.href)
                        ? 'bg-green-100 text-green-600 shadow-sm'
                        : 'text-gray-700 hover:bg-gray-100 hover:text-green-600'
                    }`}
                  >
                    <item.icon
                      className={`h-6 w-6 shrink-0 ${
                        isActive(item.href) ? 'text-green-600' : 'text-gray-500'
                      }`}
                    />
                    <span className="ml-2 text-sm leading-tight">{item.name}</span>
                  </Link>
                </li>
              ))}
            </ul>
          </nav>

          {isDemoMode ? (
            <div className="w-full mb-4 rounded-lg bg-green-100 text-green-800 text-center py-3 text-sm font-medium">
              Demo Mode
            </div>
          ) : (
            <LogoutButton />
          )}

          <div className="mt-8 flex items-center border-t border-grey-300 pt-4 space-x-4">
            <UserCircleIcon className="h-10 w-10 text-gray-500" />
            <div>
              <p className="text-sm font-medium">{username ? username : 'Guest'}</p>
              <p className="text-xs text-gray-500">Operator</p>
            </div>
          </div>
        </div>
      </aside>

      {isOpen && (
        <div
          className="fixed inset-0 bg-black opacity-50 z-40 md:hidden"
          onClick={() => setIsOpen(false)}
          aria-hidden="true"
        />
      )}
    </>
  );
};

export default Sidebar;
